#ifndef CAR_MODE_H
#define CAR_MODE_H


enum run_mode
{
    nomal_run,
    gyroleft,
    gyroright,
    chassiswith,    
} ;

enum PTZ_mode 
{
    nomal_ptz,
    relative_chassis_static,
    relative_horizon_static,
    relative_chassis_running,
    relative_horizon_running,
};

enum contrl_mode
{
    keymouse,
    remote_contrl,
    host_computer,
    quick_stoping,
};

enum connect_status
{
    waitconnecting,
    connected,
    disconnected,
    reconnceted,
};

enum conncetvalue
{
    packet_loss_low,
    packet_loss_mid,
    packet_loss_high,
    packet_slove_error,
    
};

enum chaissis_mode
{
    gyro,
    ptzwith,
    nomal_chassis,
};

enum chassis_power
{
    out_limted,
    inlimted,
    low_pwerset,
    high_powerset,
    banned,
}; 


#endif
