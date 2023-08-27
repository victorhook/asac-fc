#ifndef SERIAL_MAVLINK_H
#define SERIAL_MAVLINK_H


int serial_mavlink_init();

void serial_mavlink_broadcast_heartbeat();

void serial_mavlink_send_raw_imu();

void serial_mavlink_send_attitude();

void serial_mavlink_update();


#endif /* SERIAL_MAVLINK_H */