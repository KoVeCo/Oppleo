import os
from flask import Flask, Blueprint, render_template, abort, request, url_for, redirect, jsonify, session

from flask import current_app as app # Note: that the current_app proxy is only available in the context of a request.

from jinja2.exceptions import TemplateNotFound
from werkzeug.security import generate_password_hash, check_password_hash

from functools import wraps
import json
import logging

from flask_login import LoginManager, login_required, login_user, logout_user, current_user
from flask_socketio import SocketIO, emit

from nl.carcharging.config.WebAppConfig import WebAppConfig

from nl.carcharging.models import db
from nl.carcharging.models.User import User
from nl.carcharging.webapp.LoginForm import LoginForm
from nl.carcharging.webapp.AuthorizeForm import AuthorizeForm
from nl.carcharging.models.EnergyDeviceMeasureModel import EnergyDeviceMeasureModel
from nl.carcharging.models.Raspberry import Raspberry
from nl.carcharging.models.ChargeSessionModel import ChargeSessionModel
from nl.carcharging.models.RfidModel import RfidModel
from nl.carcharging.models.ChargerConfigModel import ChargerConfigModel
from nl.carcharging.models.EnergyDeviceModel import EnergyDeviceModel
from nl.carcharging.webapp.ChangePasswordForm import ChangePasswordForm
from nl.carcharging.webapp.RfidChangeForm import RfidChangeForm
from nl.carcharging.api.TeslaApi import TeslaAPI
from nl.carcharging.utils.UpdateOdometerTeslaUtil import UpdateOdometerTeslaUtil
from nl.carcharging.services.EvseReaderProd import EvseState
""" 
 - make sure all url_for routes point to this blueprint
"""
flaskRoutes = Blueprint('flaskRoutes', __name__, template_folder='templates')

flaskRoutesLogger = logging.getLogger('nl.carcharging.webapp.flaskRoutes')
flaskRoutesLogger.debug('Initializing routes')



@flaskRoutes.route('/', methods=['GET'])
def index():
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/ {}'.format(request.method))
    try:
        return render_template(
            'dashboard.html',
            webappconfig=WebAppConfig
            )
    except TemplateNotFound:
        abort(404)
    except Exception as e:
        flaskRoutesLogger.debug('/  - exception')
        flaskRoutesLogger.debug(e)

@flaskRoutes.route("/home")
#@authenticated_resource
def home():
    global flaskRoutesLogger
    flaskRoutesLogger.debug('/home {}'.format(request.method))
    return redirect('/')


@flaskRoutes.errorhandler(404)
def page_not_found(e):
    global flaskRoutesLogger
    flaskRoutesLogger.debug('404 page not found error handler')
    # No need for webappconfig=WebAppConfig
    return render_template('errorpages/404.html'), 404


@flaskRoutes.route('/login', methods=['GET', 'POST'])
def login():
    # For GET requests, display the login form. 
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/login {}'.format(request.method))
    if (request.method == 'GET'):
        return render_template("login.html", 
            form=LoginForm(),
            webappconfig=WebAppConfig
            )
    # For POST requests, login the current user by processing the form.
    form = LoginForm()
    flaskRoutesLogger.debug('login form created')
    if form.validate_on_submit():
        flaskRoutesLogger.debug('loginForm valid on submit')
        user = User.query.get(form.username.data)
        flaskRoutesLogger.debug('loginForm valid on submit')
        try:
            flaskRoutesLogger.debug('form.username.data = ' + form.username.data)
            if user is not None:
                flaskRoutesLogger.debug('if user:')
                if check_password_hash(user.password, form.password.data):
                    flaskRoutesLogger.debug('check_password_hash ok')
                    user.authenticated = True
                    flaskRoutesLogger.debug('place in db.session')
                    db.session.add(user)
                    db.session.commit()
                    flaskRoutesLogger.debug('login_user')
                    login_user(user, remember=form.remember_me.data)
                    if 'login_next' in session:
                        flaskRoutesLogger.debug('login_next: %s' % session['login_next'])
                        login_next = session['login_next']
                        del session['login_next']

                        from urllib.parse import urlparse
                        # only allow relative paths, so there cannot be a netloc (host)
                        if bool(urlparse(login_next).netloc):
                            # do not allow this 
                            flaskRoutesLogger.debug('login_next: {} not allowed. Using default route'.format(login_next))
                            return redirect(url_for('flaskRoutes.home'))
                        # Safe next
                        return redirect(login_next)
                    else:
                        # Return to the home page
                        flaskRoutesLogger.debug('flaskRoutes.home')
                        return redirect(url_for('flaskRoutes.home'))
        except Exception as e:
            flaskRoutesLogger.debug(type(e))
            flaskRoutesLogger.debug(e.args)
            flaskRoutesLogger.debug(e)


    flaskRoutesLogger.debug('nothing of that all')
    return render_template("login.html", 
                form=form, 
                msg="Login failed",
                webappconfig=WebAppConfig
                )

def authenticated_resource(function):
    @wraps(function)
    def decorated(*args, **kwargs):
        if (session.get('authenticated')):
            return function(*args, **kwargs)
        if (current_user.is_authenticated):
            return function(*args, **kwargs)
        # return abort(403) # unauthenticated
        # Not allowed.
        # delete old - never used - cookie
        if 'login_next' in session:
            del session['login_next']
        # if somehow ended up at logout, don't forward to login
        if (request.endpoint == "flaskRoutes.logout"):
            return redirect(url_for('flaskRoutes.home'))
        ignore_login_next = bool(request.headers.get('ignore-login-next'))
        if not ignore_login_next:
            # Redirect to login but rememmber the original request 
            session['login_next'] = request.full_path
        return redirect(url_for('flaskRoutes.login'))
    return decorated

@flaskRoutes.route("/logout", methods=["GET"])
@authenticated_resource
#@login_required
def logout():
    global flaskRoutesLogger
    flaskRoutesLogger.debug('/logout {}'.format(request.method))
    """Logout the current user."""
    user = current_user
    user.authenticated = False
    db.session.add(user)
    db.session.commit()
    logout_user()
    # return redirect(url_for('flaskRoutes.login'))
    return redirect(url_for('flaskRoutes.home'))


@flaskRoutes.route("/change_password", methods=["GET", "POST"])
@authenticated_resource
def change_password():
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/change_password {}'.format(request.method))
    if (request.method == 'GET'):
        return render_template(
            'change_password.html', 
            form=ChangePasswordForm(),
            webappconfig=WebAppConfig
            )

    form = ChangePasswordForm()
    # Validate current password
    if not check_password_hash(current_user.password, form.current_password.data):
        # Current password correct
        form.current_password.errors = []
        form.current_password.errors.append('Het huidige admin wachtwoord is niet juist.')
        return render_template(
            'change_password.html', 
            form=form,
            webappconfig=WebAppConfig
            )
    if form.validate_on_submit():
        # Valid, change the password for the user now
        user = current_user
        user.password=generate_password_hash(form.new_password.data)
        db.session.add(user)
        db.session.commit()
        return render_template(
            'change_password_success.html', 
            webappconfig=WebAppConfig
            )
    # Translate errors
    if ('new_password' in form.errors):
        for i in range(len(form.errors['new_password'])):
            if form.errors['new_password'][i] == "Field must be between 8 and 25 characters long.":
                form.errors['new_password'][i] = "Wachtwoord moet tussen 8 en 25 karakters lang zijn."
    if ('confirm_password' in form.errors):
        for i in range(len(form.errors['confirm_password'])):
            if form.errors['confirm_password'][i] == "Passwords must match":
                form.errors['confirm_password'][i] = "Wachtwoord moet gelijk zijn aan het wachtwoord hierboven."
    if ('csrf_token' in form.errors):
        for i in range(len(form.errors['csrf_token'])):
            if form.errors['csrf_token'][i] == "The CSRF token is invalid.":
                form.errors['csrf_token'][i] = "Het CSRF token is verlopen. Herlaad de pagina om een nieuw token te genereren."

    # Not valid - error message
    return render_template(
        'change_password.html', 
        form=form,
        webappconfig=WebAppConfig
        )


@flaskRoutes.route("/about")
def about():
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/about {}'.format(request.method))
    return render_template("about.html",
                webappconfig=WebAppConfig
                )


@flaskRoutes.route("/shutdown", methods=["GET", "POST"])
@flaskRoutes.route("/shutdown/", methods=["GET", "POST"])
@authenticated_resource
def shutdown():
    global flaskRoutesLogger, WebAppConfig
    # For GET requests, display the authorize form. 
    flaskRoutesLogger.debug('/shutdown {}'.format(request.method))
    if (request.method == 'GET'):
        return render_template("authorize.html", 
            form=AuthorizeForm(),
            requesttitle="Uitschakelen",
            webappconfig=WebAppConfig
            )
    # For POST requests, login the current user by processing the form.
    form = AuthorizeForm()
    if form.validate_on_submit() and \
       check_password_hash(current_user.password, form.password.data):
        flaskRoutesLogger.debug('Shutdown requested and authorized. Shutting down in 2 seconds...')
        # Simple os.system('sudo shutdown now') initiates shutdown before a webpage can be returned
        os.system("nohup sudo -b bash -c 'sleep 2; shutdown now' &>/dev/null")
        return render_template("shuttingdown.html", 
                    webappconfig=WebAppConfig
                    )
    else:
        return render_template("authorize.html", 
                form=form, 
                requesttitle="Uitschakelen",
                errormsg="Het wachtwoord is onjuist",
                webappconfig=WebAppConfig
                )


@flaskRoutes.route("/reboot", methods=["GET", "POST"])
@flaskRoutes.route("/reboot/", methods=["GET", "POST"])
@authenticated_resource
def reboot():
    global flaskRoutesLogger, WebAppConfig
    # For GET requests, display the authorize form. 
    flaskRoutesLogger.debug('/reboot {}'.format(request.method))
    if (request.method == 'GET'):
        return render_template("authorize.html", 
            form=AuthorizeForm(),
            requesttitle="Herstarten",
            webappconfig=WebAppConfig
            )
    # For POST requests, login the current user by processing the form.
    form = AuthorizeForm()
    if form.validate_on_submit() and \
       check_password_hash(current_user.password, form.password.data):
        flaskRoutesLogger.debug('Reboot requested and authorized. Rebooting in 2 seconds...')
        # Simple os.system('sudo reboot') initiates reboot before a webpage can be returned
        os.system("nohup sudo -b bash -c 'sleep 2; reboot' &>/dev/null")
        return render_template("rebooting.html", 
                    webappconfig=WebAppConfig
                    )
    else:
        return render_template("authorize.html", 
                form=form, 
                requesttitle="Herstarten",
                errormsg="Het wachtwoord is onjuist",
                webappconfig=WebAppConfig
                )


@flaskRoutes.route("/usage")
@flaskRoutes.route("/usage/")
@flaskRoutes.route("/usage/<int:cnt>")
def usage(cnt="undefined"):
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/usage ' + request.method)
    return render_template("usage_table.html", 
                cnt=cnt,
                webappconfig=WebAppConfig
                )


@flaskRoutes.route("/usage_table")
@flaskRoutes.route("/usage_table/")
@flaskRoutes.route("/usage_table/<int:cnt>")
def usage_table(cnt="undefined"):
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/usage_table {} {}'.format(cnt, request.method))
    return render_template("usage_table.html", 
                cnt=cnt,
                webappconfig=WebAppConfig
                )


@flaskRoutes.route("/usage_graph")
@flaskRoutes.route("/usage_graph/")
@flaskRoutes.route("/usage_graph/<int:cnt>")
@authenticated_resource
def usage_graph(cnt="undefined"):
    global flaskRoutesLogger, WebAppConfig
    req_period = request.args['period'] if 'period' in request.args else None
    flaskRoutesLogger.debug('/usage_graph cnt:{} method:{} type:{}'.format(cnt, request.method, req_period))
    return render_template("usage_graph.html", 
                cnt=cnt,
                req_period=req_period,
                webappconfig=WebAppConfig
                )


@flaskRoutes.route("/settings")
@flaskRoutes.route("/settings/")
@flaskRoutes.route("/settings/<int:active>")
@authenticated_resource
def settings(active=1):
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/settings {} {}'.format(active, request.method))
    diag = Raspberry().get_all()
    diag_json = json.dumps(diag)
    charger_config_str = ChargerConfigModel().get_config().to_str()
    return render_template("settings.html", 
                active=active, 
                diag=diag, 
                diag_json=diag_json,
                charger_config=charger_config_str,
                energydevicemodel=EnergyDeviceModel.get_one(WebAppConfig.ENERGY_DEVICE_ID),
                webappconfig=WebAppConfig
            )


@flaskRoutes.route("/usage_data")
@flaskRoutes.route("/usage_data/")
@flaskRoutes.route("/usage_data/<int:cnt>")
def usage_data(cnt=100):
    global flaskRoutesLogger
    flaskRoutesLogger.debug('/usage_data {} {}'.format(cnt, request.method))
    device_measurement = EnergyDeviceMeasureModel()
    device_measurement.energy_device_id = "laadpaal_noord"
    qr = device_measurement.get_last_n_saved(energy_device_id="laadpaal_noord",n=cnt)
    qr_l = []
    for o in qr:
        qr_l.append(o.to_dict())  

    return jsonify(qr_l)

# Cnt is a maximum to limit impact of this request
@flaskRoutes.route("/usage_data_since/<path:since_timestamp>")
@flaskRoutes.route("/usage_data_since/<path:since_timestamp>/<int:cnt>")
def usage_data_since(since_timestamp, cnt=-1):
    global flaskRoutesLogger
    flaskRoutesLogger.debug('/usage_data_since {} {} {}'.format(since_timestamp, cnt, request.method))
    device_measurement = EnergyDeviceMeasureModel()
    device_measurement.energy_device_id = "laadpaal_noord"
    qr = device_measurement.get_last_n_saved_since(energy_device_id="laadpaal_noord",since_ts=since_timestamp,n=cnt)
    qr_l = []
    for o in qr:
        qr_l.append(o.to_dict())  

    return jsonify(qr_l)


@flaskRoutes.route("/active_charge_session", methods=["GET"])
@flaskRoutes.route("/active_charge_session/", methods=["GET"])
@authenticated_resource
def active_charge_session():
    global WebAppConfig

    flaskRoutesLogger.debug(f'active_charge_session()')
    # Open charge session for this energy device?
    open_charge_session_for_device = \
        ChargeSessionModel.get_open_charge_session_for_device(
                WebAppConfig.ENERGY_DEVICE_ID
        )
    if open_charge_session_for_device is None:
        # None, no active session
        return jsonify({
            'status': 404, 
            'id': WebAppConfig.ENERGY_DEVICE_ID, 
            'reason': 'No active charge session'
            })
    try:
        return jsonify({ 
            'status': json.dumps(EvseState.EVSE_STATE_CHARGING) if WebAppConfig.chThread.is_status_charging else json.dumps(EvseState.EVSE_STATE_CONNECTED),
            'id': WebAppConfig.ENERGY_DEVICE_ID, 
            'data': open_charge_session_for_device.to_str() 
            })
    except Exception as e:
        pass
    return jsonify({ 
        'status': json.dump(EvseState.EVSE_STATE_UNKNOWN),
        'id': WebAppConfig.ENERGY_DEVICE_ID, 
        'reason': 'Could not determine charge session'
        })


# Cnt is a maximum to limit impact of this request
@flaskRoutes.route("/charge_sessions")
@flaskRoutes.route("/charge_sessions/")
@authenticated_resource
def charge_sessions(since_timestamp=None):
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/charge_sessions {} {}'.format(since_timestamp, request.method))
    jsonRequested = ('CONTENT_TYPE' in request.environ and 
                     request.environ['CONTENT_TYPE'].lower() == 'application/json')
    if (not jsonRequested):
        return render_template("charge_sessions.html",
                    webappconfig=WebAppConfig
                    )
    # Request parameters
    # TODO - format request params and hand to model
    req_from  = request.args['from'] if 'from' in request.args else None
    req_to    = request.args['to'] if 'to' in request.args else None
    req_limit = request.args['limit'] if 'limit' in request.args else None

    charge_sessions = ChargeSessionModel()
    charge_sessions.energy_device_id = "laadpaal_noord"
    qr = charge_sessions.get_last_n_sessions_since(
        energy_device_id="laadpaal_noord",
        since_ts=None,
        n=-1
        )
    qr_l = []
    for o in qr:
        qr_l.append(o.to_dict())
    return jsonify(qr_l)


# Cnt is a maximum to limit impact of this request
@flaskRoutes.route("/rfid_tokens", methods=["GET"])
@flaskRoutes.route("/rfid_tokens/", methods=["GET"])
@flaskRoutes.route("/rfid_tokens/<path:token>", methods=["GET", "POST"])
@flaskRoutes.route("/rfid_tokens/<path:token>/", methods=["GET", "POST"])
@authenticated_resource
def rfid_tokens(token=None):
    global flaskRoutesLogger, WebAppConfig
    flaskRoutesLogger.debug('/rfid_tokens {} {}'.format(token, request.method))
    jsonRequested = ('CONTENT_TYPE' in request.environ and 
                     request.environ['CONTENT_TYPE'].lower() == 'application/json')
    flaskRoutesLogger.debug('/rfid_tokens method: {} token: {} jsonRequested: {}'.format(request.method, token, jsonRequested))
    if (not jsonRequested):
        if (token == None):
            return render_template(
                'tokens.html',
                webappconfig=WebAppConfig
                )
        rfid_model = RfidModel().get_one(token)
        if (rfid_model == None):
            return render_template(
                'tokens.html', 
                webappconfig=WebAppConfig
            )
        rfid_model.cleanupOldToken()    # Remove any expired token info
        rfid_change_form = RfidChangeForm()
        flaskRoutesLogger.debug('CSRF Token: {}'.format(rfid_change_form.csrf_token.current_token) )
        if (request.method == 'POST'):
            # Update for specific token
            if rfid_change_form.validate_on_submit():
                # rfid - given
                rfid_model.enabled = rfid_change_form.enabled.data
                # created_at - updated by the system
                # last_used_at - updated by the system
                rfid_model.name = rfid_change_form.name.data
                rfid_model.vehicle_make = rfid_change_form.vehicle_make.data
                rfid_model.vehicle_model = rfid_change_form.vehicle_model.data
                # only accept odometer if token and id
                rfid_model.get_odometer = (rfid_change_form.get_odometer.data and 
                                           rfid_model.hasValidToken() and 
                                           rfid_change_form.vehicle_id.data is not None and
                                           len(rfid_change_form.vehicle_id.data) > 0)
                rfid_model.license_plate = rfid_change_form.license_plate.data
                rfid_model.valid_from = rfid_change_form.valid_from.data
                rfid_model.valid_until = rfid_change_form.valid_until.data
                # api_access_token - updated via ajax
                # api_token_type - updated via ajax
                # api_created_at - updated via ajax
                # api_expires_in - updated via ajax
                # api_refresh_token - updated via ajax
                rfid_model.vehicle_name = None if rfid_change_form.vehicle_name.data is None or \
                                                  len(rfid_change_form.vehicle_name.data) == 0 \
                                               else rfid_change_form.vehicle_name.data
                rfid_model.vehicle_id = None if rfid_change_form.vehicle_id.data is None or \
                                                len(rfid_change_form.vehicle_id.data) == 0 \
                                             else rfid_change_form.vehicle_id.data
                rfid_model.vehicle_vin = None if rfid_change_form.vehicle_vin.data is None or \
                                                 len(rfid_change_form.vehicle_vin.data) == 0 \
                                              else rfid_change_form.vehicle_vin.data
                rfid_model.save()
                # Return to the rfid tokens page
                return redirect(url_for('flaskRoutes.rfid_tokens', req_format='html', token=None))
            # TODO - what went wrong? message!
            return render_template("token.html",
                    rfid_model=rfid_model,
                    form=rfid_change_form,
                    webappconfig=WebAppConfig
                )        
        return render_template("token.html",
                    rfid_model=rfid_model,
                    form=rfid_change_form,
                    webappconfig=WebAppConfig
                )        
    # Check if token exist, if not rfid is None
    if (token == None):
        rfid_list = []
        rfid_models = RfidModel().get_all()
        for rfid_model in rfid_models:
            rfid_model.cleanupOldToken()    # Remove any expired token info
            rfid_list.append(rfid_model.to_str())
        return jsonify(rfid_list)
    # Specific token
    rfid_model = RfidModel().get_one(token)
    if (rfid_model == None):
        return jsonify({})
    rfid_model.cleanupOldToken()    # Remove any expired token info
    return jsonify(rfid_model.to_str())

# Always returns json
@flaskRoutes.route("/rfid_tokens/<path:token>/TeslaAPI/GenerateOAuth", methods=["POST"])
@authenticated_resource  # CSRF Token is valid
def TeslaApi_GenerateOAuth(token=None):
    global flaskRoutesLogger
    flaskRoutesLogger.debug('/rfid_tokens/{}/TeslaAPI/GenerateOAuth {}'.format(token, request.method))
    flaskRoutesLogger.debug('/rfid_tokens/{}/TeslaAPI/GenerateOAuth method: {} token: {}'.format(token, request.method, token))
    rfid_model = RfidModel().get_one(token)
    if ((token == None) or (rfid_model == None)):
        # Nope, no token
        return jsonify({
            'status': 404, 
            'reason': 'No known RFID token'
            })

    # Update for specific token
    tesla_api = TeslaAPI()
    if tesla_api.authenticate(
        email=request.form['oauth_email'], 
        password=request.form['oauth_password']):
        # Obtained token
        UpdateOdometerTeslaUtil.copy_token_from_api_to_rfid_model(tesla_api, rfid_model)
        rfid_model.save()
        return jsonify({
            'status': 200, 
            'token_type': rfid_model.api_token_type, 
            'created_at': rfid_model.api_created_at, 
            'expires_in': rfid_model.api_expires_in,
            'vehicles' : tesla_api.getVehicleNameIdList()
            })
    else:
        # Nope, no token
        return jsonify({
            'status': 401,  
            'reason': 'Not authorized'
            })

# Always returns json
@flaskRoutes.route("/rfid_tokens/<path:token>/TeslaAPI/RefreshOAuth", methods=["POST"])
@authenticated_resource
def TeslaApi_RefreshOAuth(token=None):
    global flaskRoutesLogger
    flaskRoutesLogger.debug('/rfid_tokens/{}/TeslaAPI/RefreshOAuth {}'.format(token, request.method))
    flaskRoutesLogger.debug('/rfid_tokens/{}/TeslaAPI/RefreshOAuth method: {} token: {}'.format(token, request.method, token))
    rfid_model = RfidModel().get_one(token)
    if ((token == None) or (rfid_model == None)):
        # Nope, no token
        return jsonify({
            'status': 404, 
            'reason': 'No known RFID token'
            })
    # Update for specific token
    tesla_api = TeslaAPI()
    UpdateOdometerTeslaUtil.copy_token_from_rfid_model_to_api(rfid_model, tesla_api)
    if not tesla_api.refreshToken():
        return jsonify({
            'status': 500,
            'reason': 'Refresh failed'
            })
    # Refresh succeeded, Obtained token
    UpdateOdometerTeslaUtil.copy_token_from_api_to_rfid_model(tesla_api, rfid_model)
    rfid_model.save()

    return jsonify({
        'status': 200, 
        'token_type': rfid_model.api_token_type, 
        'created_at': rfid_model.api_created_at, 
        'expires_in': rfid_model.api_expires_in,
        'vehicles' : tesla_api.getVehicleNameIdList()
        })

# Always returns json
@flaskRoutes.route("/rfid_tokens/<path:token>/TeslaAPI/RevokeOAuth", methods=["POST"])
@authenticated_resource
def TeslaApi_RevokeOAuth(token=None):
    global flaskRoutesLogger
    flaskRoutesLogger.debug('/rfid_tokens/{}/TeslaAPI/RevokeOAuth {}'.format(token, request.method))
    flaskRoutesLogger.debug('/rfid_tokens/{}/TeslaAPI/RevokeOAuth method: {} token: {}'.format(token, request.method, token))
    rfid_model = RfidModel().get_one(token)
    if ((token == None) or (rfid_model == None)):
        # Nope, no token
        return jsonify({
            'status': 404, 
            'reason': 'No known RFID token'
            })
    # Update for specific token
    tesla_api = TeslaAPI()
    UpdateOdometerTeslaUtil.copy_token_from_rfid_model_to_api(rfid_model, tesla_api)
    if not tesla_api.revokeToken():
        return jsonify({
            'status': 500,
            'reason': 'Revoke failed'
            })
    # Revoke succeeded, remove token
    UpdateOdometerTeslaUtil.clean_token_rfid_model(rfid_model)
    rfid_model.save()

    return jsonify({
        'status': 200, 
        'token_type': '', 
        'created_at': '', 
        'expires_in': '',
        'vehicles' : ''
        })
