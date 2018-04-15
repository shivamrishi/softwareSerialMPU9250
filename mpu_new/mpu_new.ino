#define _csPin 3
#define _miso 12
#define _mosi 11
#define _sck 13
#define USER_CTRL 0x6A
#define I2C_MST_EN 0x20
#define I2C_MST_CTRL 0x24 
#define I2C_MST_CLK 0x0D
#define AK8963_CNTL1 0x0A
#define AK8963_PWR_DOWN 0x00
#define PWR_MGMNT_1 0x6B
#define PWR_RESET 0x80
#define AK8963_CNTL2 0x0B
#define AK8963_RESET 0x01
#define AK8963_FUSE_ROM 0x0F
#define AK8963_ASA 0x10
//#define _magScaleX 0.18
//#define _magScaleY 0.18
#define AK8963_CNT_MEAS1 0x12
#define AK8963_CNT_MEAS2 0x16 
#define AK8963_HXL 0x03
#define I2C_SLV0_ADDR 0x25
#define AK8963_I2C_ADDR 0x0C
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_DO 0x63
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_EN 0x80
#define I2C_READ_FLAG 0x80
#define EXT_SENS_DATA_00 0x49
#define SPI_READ 0x80
#define WHO_AM_I 0x75
#define AK8963_WHO_AM_I 0x00
#define ACCEL_OUT  0x3B
float _hxs = 1.0f;
float _hys = 1.0f;
//float _magScaleX 0.18
//float _magScaleY 0.18
int16_t _hxcounts;
int16_t _hycounts;
float _hx;
float _hy;
int _status;
struct magData{
  float HX;
  float HY;
};
uint8_t _buffer[4];
byte SPI_transfer(byte);
int readRegisters(uint8_t, uint8_t, uint8_t*);
int writeRegisters(uint8_t, uint8_t);
int writeAK8963Register(uint8_t, uint8_t);
int readAK8963Registers(uint8_t , uint8_t , uint8_t* );
int Sensorbegin(void);
magData getValues(void);
int whoAmIAK8963(void);
int whoAmI(void);

int test(){
   pinMode(_csPin,OUTPUT);
    digitalWrite(_csPin,HIGH);
    pinMode(_miso,INPUT);
    pinMode(_mosi,OUTPUT);
    pinMode(_sck,OUTPUT);
  if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
    return -2;
  }else{ Serial.println("successful"); return 0;}
}
byte SPI_transfer(byte input){

for (int i = 0; i < 8; i++) {
        digitalWrite(_sck, LOW);// falling edge to initiate transfer
        if(input & 0x80){
        digitalWrite(_mosi, HIGH);}//transition the msb
        else{ digitalWrite(_mosi,LOW);}
        digitalWrite(_sck, HIGH);//latch the input
        input = input<<1;
        if(digitalRead(_miso)) //read the output
       { input |= 0x01;}
        else{
        input &= 0xFE;}
       }
return input;
}
int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
    digitalWrite(_csPin,LOW); // select the MPU9250 chip
    delay(100); //delay for slave select
    SPI_transfer(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < count; i++){
      dest[i] = SPI_transfer(0x01); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
        return 1;
}
int writeRegister(uint8_t subAddress, uint8_t data){
  /* write data to device */
    digitalWrite(_sck,HIGH);
    digitalWrite(_csPin,LOW); // select the MPU9250 chip
    SPI_transfer(subAddress); // write the register address
    SPI_transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    delay(10);
    digitalWrite(_sck,HIGH);
  /* read back the register */
  readRegisters(subAddress,1,_buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

int writeAK8963Register(uint8_t subAddress, uint8_t data){
  // set slave 0 to the AK8963 and set for write
  if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address 
  if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // store the data for write
  if (writeRegister(I2C_SLV0_DO,data) < 0) {
    return -3;
  }
  // enable I2C and send 1 byte
  if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
    return -4;
  }
  // read the register and confirm
  if (readAK8963Registers(subAddress,1,_buffer) < 0) {
    return -5;
  }
  if(_buffer[0] == data) {
    return 1;
  } else{
    return -6;
  }
}
/* reads registers from the AK8963 */
int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
  // set slave 0 to the AK8963 and set for read
  if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
  if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // enable I2C and request the bytes
  if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
    return -3;
  }
  delay(1); // takes some time for these registers to fill
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  _status = readRegisters(EXT_SENS_DATA_00,count,dest); 
  return _status;
}
int Sensorbegin(){
    // setting CS pin to output
    pinMode(_csPin,OUTPUT);
    digitalWrite(_csPin,HIGH);
    pinMode(_miso,INPUT);
    pinMode(_mosi,OUTPUT);
    pinMode(_sck,OUTPUT);
  // enable I2C master mode
  if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
    return -2;
  }else{ Serial.println("successful");}
  // set the I2C bus speed to 400 kHz
  if(writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
    return -3;
  }
  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  delay(1);
  // reset the AK8963
  writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  if((whoAmI() != 113)&&(whoAmI() != 115)){
    return -5;
  }
  // enable I2C master mode
  if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
    return -12;
  }
  // set the I2C bus speed to 400 kHz
  if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
    return -13;
  }
  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
  if( whoAmIAK8963() != 72 ){
    return -14;
  }
  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -15;
  }
  delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if(writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
    return -16;
  }
  delay(100); // long wait between AK8963 mode changes
  if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -17; }
  delay(100); // long wait between AK8963 mode changes  
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
    return -18;
  }
  delay(100); // long wait between AK8963 mode changes 
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(AK8963_HXL,7,_buffer);
  // successful init, return 1
  return 1;
}
int whoAmI(){
  // read the WHO AM I register
  if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int whoAmIAK8963(){
  // read the WHO AM I register
  if (readAK8963Registers(AK8963_WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

magData getValues() {
  magData r;
  r = {-1,-1};
  if (readRegisters(0x49, 1, _buffer) < 0) {
    return r;
  }

  if (readRegisters(0x4A, 1, _buffer+1) < 0) {
    return r;
  }
  
  if (readRegisters(0x4B, 1, _buffer+2) < 0) {
    return r;
  }
  
  if (readRegisters(0x4C, 1, _buffer+3) < 0) {
    return r;
  }
// combine into 16 bit value
  _hxcounts = (((int16_t)_buffer[1]) << 8) | _buffer[0];
  _hycounts = (((int16_t)_buffer[3]) << 8) | _buffer[2];
  _hx = (((float)(_hxcounts) * 0.18))*1.0f;
  _hy = (((float)(_hycounts) * 0.18))*1.0f;

r.HX = _hx;
r.HY = _hy;
  return r;
}

void getDirValues(int count)
{

for( int i =0;i<count;i++){
magData d = getValues();
Serial.print(static_cast<int>(d.HX));
Serial.print('\t');
Serial.println(static_cast<int>(d.HY));
}

}

void serialFlush(){
  while(Serial.available()){ Serial.read();}
}
void acquireValues(float no_dir, int samples){
int steps = 360/no_dir; 
char doOperation;
for(int i =0;i< steps; i++)
{     Serial.print(" move in direction ") ;
      Serial.println(i*no_dir);
      serialFlush();
      while(Serial.available()<=0){} 
      getDirValues(samples);
}

return;  
}


int status;
void setup() {
   Serial.begin(115200);
  while(!Serial) {}
    test();
  // start communication with IMU 
    status = Sensorbegin();
    if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
acquireValues(11.25,400);
}

void loop() {
// put your main code here, to run repeatedly:
}
