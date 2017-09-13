# LSM6DSL
Arduino library to support the LSM6DSL 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C to communicate. It is then required to create a TwoWire interface before accessing to the sensors:  

    dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL);  
    dev_i2c->begin();  

An instance can be created and enabled following the procedure below:  

    AccGyr = new LSM6DSLSensor(dev_i2c);  
    AccGyr->Enable_X();  
    AccGyr->Enable_G();  

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    AccGyr->Get_X_Axes(accelerometer);  
    AccGyr->Get_G_Axes(gyroscope);  

## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSL

The LSM6DSL datasheet is available at  
http://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsl.html
