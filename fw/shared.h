typedef struct
{
    float temp;
    float humidity;
    float pressure;
    float gas_resistance;
    float v_batt;
    float i_batt;
    float v_solar;
    float i_solar;
    float v_load;
    float i_load;
    char rain_count;
    char wind_speed_count;
    int raw_wind_dir;
    u_int64_t uptime;
} data_struct;
