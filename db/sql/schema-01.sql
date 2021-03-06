create table session (
    id serial primary key,
    rfid varchar(100) not null,
    start_value float,
    end_value float,
    created_at timestamp,
    modified_at timestamp
);

create table energy_device (
    energy_device_id varchar(100) primary key,
    port_name varchar(100),
    slave_address int
);

insert into energy_device values ('device_1', '/dev/ttyUSB0', 1);

create table energy_device_measures (
    id serial primary key,
    energy_device_id varchar(100) references energy_device(energy_device_id),
    kwh_l1 float,
    kwh_l2 float,
    kwh_l3 float,
    a_l1 float,
    a_l2 float,
    a_l3 float,
    v_l1 float,
    v_l2 float,
    v_l3 float,
    p_l1 float,
    p_l2 float,
    p_l3 float,
    kw_total float,
    hz float,
    created_at timestamp
);
