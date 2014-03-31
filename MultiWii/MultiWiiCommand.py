import serial
import time
import struct
#import sys

#/******************************** MultiWii Serial Protocol******************************/
class MultiWiiSP:
   # Shared class variables....
   MSP_HEADER = "$M<"
   
   MSP_NONE            = 0     #This is a flag for error situations

   MSP_IDENT           = 100   #out message   multitype + multiwii version + protocol version + capability variable
   MSP_STATUS          = 101   #out message   cycletime & errors_count & sensor present & box activation & current setting number
   MSP_RAW_IMU         = 102   #out message   9 DOF
   MSP_SERVO           = 103   #out message   8 servos
   MSP_MOTOR           = 104   #out message   8 motors
   MSP_RC              = 105   #out message   8 rc chan and more
   MSP_RAW_GPS         = 106   #out message   fix, numsat, lat, lon, alt, speed, ground course
   MSP_COMP_GPS        = 107   #out message   distance home, direction home
   MSP_ATTITUDE        = 108   #out message   2 angles 1 heading
   MSP_ALTITUDE        = 109   #out message   altitude, variometer
   MSP_BAT             = 110   #out message   vbat, powermetersum, rssi if available on RX
   MSP_RC_TUNING       = 111   #out message   rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
   MSP_PID             = 112   #out message   P I D coeff (9 are used currently)
   MSP_BOX             = 113   #out message   BOX setup (number is dependant of your setup)
   MSP_MISC            = 114   #out message   powermeter trig
   MSP_MOTOR_PINS      = 115   #out message   which pins are in use for motors & servos, for GUI
   MSP_BOXNAMES        = 116   #out message   the aux switch names
   MSP_PIDNAMES        = 117   #out message   the PID names
   MSP_WP              = 118   #out message   get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
   MSP_BOXIDS          = 119   #out message   get the permanent IDs associated to BOXes
   MSP_SERVO_CONF      = 120   #out message   Servo settings
   MSP_NAV_STATUS      = 121   #out message   Navigation
   MSP_NAV_CONFIG      = 122   #out message
   
   MSP_RADIO           = 199   #Special message inserted by 3Dr radio for Multiwii
   
   MSP_SET_RAW_RC      = 200   #in message    8 rc chan
   MSP_SET_RAW_GPS     = 201   #in message    fix, numsat, lat, lon, alt, speed
   MSP_SET_PID         = 202   #in message    P I D coeff (9 are used currently)
   MSP_SET_BOX         = 203   #in message    BOX setup (number is dependant of your setup)
   MSP_SET_RC_TUNING   = 204   #in message    rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
   MSP_ACC_CALIBRATION = 205   #in message    no param
   MSP_MAG_CALIBRATION = 206   #in message    no param
   MSP_SET_MISC        = 207   #in message    powermeter trig + 8 free for future use
   MSP_RESET_CONF      = 208   #in message    no param
   MSP_SET_WP          = 209   #in message    sets a given WP (WP#,lat, lon, alt, flags)
   MSP_SELECT_SETTING  = 210   #in message    Select Setting Number (0-2)
   MSP_SET_HEAD        = 211   #in message    define a new heading hold direction
   MSP_SET_SERVO_CONF  = 212   #in message    Servo settings
   MSP_SET_MOTOR       = 214   #in message    PropBalance function
   MSP_SET_NAV_CONFIG  = 215   #
   
   MSP_BIND            = 240   #in message    no param
   
   MSP_EEPROM_WRITE    = 250   #in message    no param
   
   MSP_DEBUGMSG        = 253   #out message   debug string buffer
   MSP_DEBUG           = 254   #out message   debug1,debug2,debug3,debug4
   
   IDLE = 0
   HEADER_START = 1
   HEADER_M = 2
   HEADER_ARROW = 3
   HEADER_SIZE = 4
   HEADER_CMD = 5
   HEADER_ERR = 6


   CHECKBOXITEMS=0 # Read from the device
   PIDITEMS=10

   def __init__(self):
      usbPort = '/dev/ttyUSB0'
      GUI_BaudRate = 115200        # Default.
      self.sc = serial.Serial(port=usbPort, baudrate=GUI_BaudRate, timeout=3) # Serial g_serial;
      #self.sc = serial.Serial(usbPort, timeout=3) # Serial g_serial;
      self.c_state = self.IDLE     # int
      self.err_rcvd = False   # boolean

      self.checksum=0             # byte
      self.cmd=0                  # byte
      self.offset=0               # int
      self.dataSize=0             # int
      self.inBuf = bytearray()    # 256 bytes
      self.p = int() # int
      # MSP_IDENT
      self.version=0
      self.versionMisMatch=0
      self.multiType = int()  # 1 for tricopter, 2 for quad+, 3 for quadX, ...
      self.multiCapability = int(0) # Bitflags stating what capabilities are/are not present in the compiled code.
      # MSP_STATUS
      self.mode = int()
      self.cycleTime=0
      self.i2cError=0
      self.present = 0
      # MSP_RAW_IMU
      self.gx=0.0
      self.gy=0.0
      self.gz=0.0
      self.ax=0.0
      self.ay=0.0
      self.az=0.0
      self.magx=0.0
      self.magy=0.0
      self.magz=0.0
      # MSP_SERVO
      self.servo = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      #servo = array('f', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
      # MSP_MOTOR
      self.mot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      #mot = array('f', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
      # MSP_RC
      # RC control positions
      self.rcThrottle = 1500
      self.rcRoll = 1500
      self.rcPitch = 1500
      self.rcYaw =1500
      self.rcAUX1=1500
      self.rcAUX2=1500
      self.rcAUX3=1500
      self.rcAUX4=1500
      # MSP_RAW_GPS
      self.GPS_fix=0
      self.GPS_numSat=0
      self.GPS_latitude=0
      self.GPS_longitude=0
      self.GPS_speed=0
      self.GPS_altitude=0
      # MSP_COMP_GPS
      self.GPS_distanceToHome=0
      self.GPS_directionToHome=0
      self.GPS_update=0
      # MSP_ATTITUDE
      self.angx=0.0
      self.angy=0.0
      self.head=0.0
      # MSP_ALTITUDE
      self.alt=0.0
      # MSP_BAT
      self.bytevbat=0
      self.pMeterSum=0
      # MSP_RC_TUNING
      self.byteRC_RATE=0
      self.byteRC_EXPO=0
      self.byteRollPitchRate=0
      self.byteYawRate=0
      self.byteDynThrPID=0
      self.byteThrottle_EXPO=0
      self.byteThrottle_MID=0
      self.byteSelectSetting=0
      # MSP_ACC_CALIBRATION
      # MSP_MAG_CALIBRATION
      # MSP_PID
      self.byteP = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      self.byteI = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      self.byteD = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      # MSP_BOX
      self.activation = [0, 0]
      # CheckBox checkbox[];
      #activation_items = 2 # Use len(activation_items) to get length
      # MSP_BOXNAMES
      self.boxNames = ['string1', 'string2']
      #boxItems = 2 # Use len(boxNames) to get length
      # MSP_PIDNAMES
      # MSP_MISC
      self.intPowerTrigger=0
      # MSP_MOTOR_PINS
      self.byteMP = [1, 2, 3, 4, 5, 6, 7, 8]  # Motor Pins. Varies by multiType and Arduino model (pro Mini, Mega, etc).
      # MSP_DEBUGMSG
      # MSP_DEBUG
      self.debug1=0.0
      self.debug2=0.0
      self.debug3=0.0
      self.debug4=0.0
      self.format32 = struct.Struct('@i')
      self.format16 = struct.Struct('@h')
      self.format8 = struct.Struct('@b')

   def __del__(self):
      self.sc.close()
      print "Connection to MultiWii closed."

   def read32(self): # int
      num = (self.format32.unpack(str(self.inBuf[self.p:(self.p + 4)])))[0]
      #num = ((self.inBuf[self.p]     & 0xff) +
      #      ((self.inBuf[self.p + 1] & 0xff) << 8) +
      #      ((self.inBuf[self.p + 2] & 0xff) << 16) +
      #      ((self.inBuf[self.p + 3] & 0xff) << 24))
      self.p += 4
      #if num & 0x80000000:
      #   num = -0x80000000 + num
      return num

   def read16(self): # int
      num = (self.format16.unpack(str(self.inBuf[self.p:(self.p + 2)])))[0]
      #num = ((self.inBuf[self.p]     & 0xff) +
      #      ((self.inBuf[self.p + 1] & 0xff) << 8))
      self.p += 2
      #if num & 0x8000:
      #   num = -0x8000 + num
      return num

   def read8(self): # int
      num = (self.format8.unpack(str(self.inBuf[self.p])))[0]
      #num = (self.inBuf[self.p] & 0xff)
      self.p += 1
      #if num & 0x80:
      #   num = -0x80 + num
      return num

   def Msp_Ident(self, datasize):
      version = self.read8()
      multiType = self.read8()
      self.read8()                     # MSP version
      multiCapability = self.read32()  # capability
      #if ((multiCapability&1)>0) {
      #   buttonRXbind = controlP5.addButton("bRXbind",1,10,yGraph+205-10,55,10);
      #   buttonRXbind.setColorBackground(blue_);
      #   buttonRXbind.setLabel("RX Bind");

   def Msp_Status(self, datasize):
      cycleTime = self.read16()
      i2cError = self.read16()
      present = self.read16()
      mode = self.read32()
      #if ((present&1) >0)
      #  {buttonAcc.setColorBackground(green_);}
      #else
      #  {buttonAcc.setColorBackground(red_);
      #  tACC_ROLL.setState(false);
      #  tACC_PITCH.setState(false);
      #  tACC_Z.setState(false);}
      #if ((present&2) >0)
      #  {buttonBaro.setColorBackground(green_);}
      #else
      #  {buttonBaro.setColorBackground(red_);
      #  tBARO.setState(false); }
      #if ((present&4) >0)
      #  {buttonMag.setColorBackground(green_);}
      #else
      #  {buttonMag.setColorBackground(red_);
      #  tMAGX.setState(false);
      #  tMAGY.setState(false);
      #  tMAGZ.setState(false); }
      #if ((present&8) >0)
      #  {buttonGPS.setColorBackground(green_);}
      #else
      #  {buttonGPS.setColorBackground(red_);
      #  tHEAD.setState(false);}
      #if ((present&16)>0)
      #  {buttonSonar.setColorBackground(green_);}
      #else
      #  {buttonSonar.setColorBackground(red_);}
      #for(i=0;i<CHECKBOXITEMS;i++) {
      #  if ((mode&(1<<i))>0)
      #    buttonCheckbox[i].setColorBackground(green_);
      #  else
      #    buttonCheckbox[i].setColorBackground(red_);
      #}
      #confSetting.setValue(read8());
      #confSetting.setColorBackground(green_);

   def Msp_Raw_Imu(self, datasize):
      # RAW Data from MultiWii
      self.ax = self.read16()
      self.ay = self.read16()
      self.az = self.read16()
      self.gx = self.read16()/8
      self.gy = self.read16()/8
      self.gz = self.read16()/8
      self.magx = self.read16()/3
      self.magy = self.read16()/3
      self.magz = self.read16()/3

   def Msp_Servo(self, datasize):
      for i in range(0, 8):
         self.servo[i] = self.read16()

   def Msp_Motor(self, datasize):
      for i in range(0, 8):
         self.mot[i] = self.read16()

   def Msp_Rc(self, datasize):
      # RC/Input position data from MultiWii
      self.rcRoll = self.read16()
      self.rcPitch = self.read16()
      self.rcYaw = self.read16()
      self.rcThrottle = self.read16();
      self.rcAUX1 = self.read16()
      self.rcAUX2 = self.read16()
      self.rcAUX3 = self.read16()
      self.rcAUX4 = self.read16()

   def Msp_Raw_Gps(self, datasize):
      self.GPS_fix = self.read8()
      self.GPS_numSat = self.read8()
      self.GPS_latitude = self.read32()
      self.GPS_longitude = self.read32()
      self.GPS_altitude = self.read16()
      self.GPS_speed = self.read16()

   def Msp_Comp_Gps(self, datasize):
      self.GPS_distanceToHome = self.read16()
      self.GPS_directionToHome = self.read16()
      self.GPS_update = self.read8()

   def Msp_Attitude(self, datasize):
      # Processed Compass Data
      self.angx = self.read16()/10
      self.angy = self.read16()/10
      self.head = self.read16()

   def Msp_Altitude(self, datasize):
      # Processed Altitude data
      self.alt = self.read32()

   def Msp_Bat(self, datasize):
      self.bytevbat = self.read8()
      self.pMeterSum = self.read16()

   def Msp_Rc_Tuning(self, datasize):
      # Configuration Data - Only needs to be read once
      self.byteRC_RATE = self.read8()
      self.byteRC_EXPO = self.read8()
      self.byteRollPitchRate = self.read8()
      self.byteYawRate = self.read8()
      self.byteDynThrPID = self.read8()
      self.byteThrottle_MID = self.read8()
      self.byteThrottle_EXPO = self.read8()
      # ONLY RELATED TO GUI DISPLAY
      #confRC_RATE.setValue(byteRC_RATE/100.0);
      #confRC_EXPO.setValue(byteRC_EXPO/100.0);
      #rollPitchRate.setValue(byteRollPitchRate/100.0);
      #yawRate.setValue(byteYawRate/100.0);
      #dynamic_THR_PID.setValue(byteDynThrPID/100.0);
      #throttle_MID.setValue(byteThrottle_MID/100.0);
      #throttle_EXPO.setValue(byteThrottle_EXPO/100.0);
      #confRC_RATE.setColorBackground(green_);
      #confRC_EXPO.setColorBackground(green_);
      #rollPitchRate.setColorBackground(green_);
      #yawRate.setColorBackground(green_);
      #dynamic_THR_PID.setColorBackground(green_);
      #throttle_MID.setColorBackground(green_);
      #throttle_EXPO.setColorBackground(green_);
      #updateModelMSP_SET_RC_TUNING();

   def Msp_Acc_Calibration(self, datasize):
      # Not Implemented
      return 0

   def Msp_Mag_Calibration(self, datasize):
      # Not Implemented
      return 0

   def Msp_Pid(self, datasize):
      for i in range(self.PIDITEMS):
         self.byteP[i] = self.read8()
         self.byteI[i] = self.read8()
         self.byteD[i] = self.read8()
      # ONLY RELATED TO GUI DISPLAY
      #   #Different rates fot POS-4 POSR-5 NAVR-6
      #   if (i >= 0 and i <= 3) or (i >= 7 and i <= 9):
      #      confP[i].setValue(byteP[i]/10.0);
      #      confI[i].setValue(byteI[i]/1000.0);
      #      confD[i].setValue(byteD[i]);
      #   elif i == 4:
      #      confP[i].setValue(byteP[i]/100.0)
      #      confI[i].setValue(byteI[i]/100.0)
      #      confD[i].setValue(byteD[i]/1000.0)
      #   elif i == 5 or i == 6:
      #      confP[i].setValue(byteP[i]/10.0)
      #      confI[i].setValue(byteI[i]/100.0)
      #      confD[i].setValue(byteD[i]/1000.0)
      #   confP[i].setColorBackground(green_);
      #   confI[i].setColorBackground(green_);
      #   confD[i].setColorBackground(green_);
      #updateModelMSP_SET_PID();

   def Msp_Box(self, datasize):
      del self.activation
      for i in range(self.CHECKBOXITEMS):
         self.activation.append(self.read16());
      # ONLY RELATED TO GUI DISPLAY
      #   for(int aa=0;aa<12;aa++) {
      #      if ((activation[i]&(1<<aa))>0)
      #         checkbox[i].activate(aa);
      #      else
      #         checkbox[i].deactivate(aa);


   def Msp_Boxnames(self, datasize):
      del self.boxNames
      self.boxItems = 0
      # Perform some string magic, re-populate boxNames...
      strList = ((self.inBuf[0:datasize]).decode()).split(';')
      for str in strList:
         self.boxNames.append(str)
      self.CHECKBOXITEMS = len(self.boxNames)
      # ONLY RELATED TO GUI DISPLAY
      #create_checkboxes(new String(inBuf, 0, dataSize).split(";"));

   def Msp_Pidnames(self, datasize):
      #/* TODO create GUI elements from this message */
      #System.out.println("Got PIDNAMES: "+new String(inBuf, 0, dataSize));
      return 0

   def Msp_Misc(self, datasize):
      self.intPowerTrigger = self.read16()
      # ONLY RELATED TO GUI DISPLAY
      #confPowerTrigger.setValue(intPowerTrigger);
      #updateModelMSP_SET_MISC();

   def Msp_Motor_Pins(self, datasize):
      for i in range(0, 8):
         self.byteMP[i] = self.read8()


   def Msp_Debugmsg(self, datasize):
      while dataSize > 0:
         datasize = datasize - 1
         c = str(self.read8())
         if c != 0:
            print c

   def Msp_Debug(self, datasize):
      self.debug1 = self.read16();
      self.debug2 = self.read16();
      self.debug3 = self.read16();
      self.debug4 = self.read16();
   
   def Msp_Set_Raw_Rc(self, datasize):
      #print "Set RC Reply"
      return 0

   ######### Dictionary to allow extra flow control #####################
   switch = { MSP_IDENT :           Msp_Ident           ,
              MSP_STATUS :          Msp_Status          ,
              MSP_RAW_IMU :         Msp_Raw_Imu         ,
              MSP_SERVO :           Msp_Servo           ,
              MSP_MOTOR :           Msp_Motor           ,
              MSP_RC :              Msp_Rc              ,
              MSP_RAW_GPS :         Msp_Raw_Gps         ,
              MSP_COMP_GPS :        Msp_Comp_Gps        ,
              MSP_ATTITUDE :        Msp_Attitude        ,
              MSP_ALTITUDE :        Msp_Altitude        ,
              MSP_BAT :             Msp_Bat             ,
              MSP_RC_TUNING :       Msp_Rc_Tuning       ,
              MSP_SET_RAW_RC :      Msp_Set_Raw_Rc      ,
              MSP_ACC_CALIBRATION : Msp_Acc_Calibration ,
              MSP_MAG_CALIBRATION : Msp_Mag_Calibration ,
              MSP_PID :             Msp_Pid             ,
              MSP_BOX :             Msp_Box             ,
              MSP_BOXNAMES :        Msp_Boxnames        ,
              MSP_PIDNAMES :        Msp_Pidnames        ,
              MSP_MISC :            Msp_Misc            ,
              MSP_MOTOR_PINS :      Msp_Motor_Pins      ,
              MSP_DEBUGMSG :        Msp_Debugmsg        ,
              MSP_DEBUG :           Msp_Debug
   }

   def evaluateCommand(self, cmd, dataSize):
      icmd = int(cmd & 0xFF)
      #print "ExecCommand"
      self.switch[icmd](self, dataSize)
      #default:
      #println("Don't know how to handle reply "+icmd);

   ##############################################################################
   ## Main work is from here to the next set of ##############
   
   #send msp with payload
   def requestFormatMSP_WP(self, msp, payload):  # int, Character[]; return list<byte>
      if(msp < 0):
         return []
      bf = bytearray()
      for c in MultiWiiSP.MSP_HEADER:
         bf.append( c )
      
      checksum=0
      pl_size = 0
      if payload != None:
         pl_size = len(payload) & 0xFF
      bf.append(pl_size & 0xFF)
      checksum = checksum ^ (pl_size&0xFF)
      
      bf.append(msp & 0xFF)
      checksum = checksum ^ (msp&0xFF)
      
      if (payload != None):
         for c in payload:
            bf.append( ord(c) & 0xFF )
            checksum = checksum ^ ( ord(c) & 0xFF )
      bf.append(checksum & 0xFF)
      return bf
   
   #send msp without payload
   def requestMSP(self, msp): #private List<Byte> requestMSP(int msp) {
      return self.requestFormatMSP_WP(msp, None)
    
   #send multiple msp without payload
   def requestsMSP(self, msps): #private List<Byte> requestsMSP (int[] msps) {
      s = bytearray()
      for m in msps:
         s += (self.requestFormatMSP_WP(m, None))
      return s
   
   # Send MSP request
   def sendRequestMSP(self, msp):
      arr = bytearray()
      i = 0
      for b in msp:
         arr.append(b & 0xFF)
         i += 1
      #print "Request string: ","len: ",len(arr),": ",arr
      self.sc.write(arr); # send the complete byte sequence in one go
      #self.sc.write(chr(msp[0]))

####################################################
# Data Requests
#void draw() {
#  List<Character> payload;
#  int i,aa;
#  float val,inter,a,b,h;
#  int c;
#  if (init_com==1 && graph_on==1) {
#    time=millis();
#
#    if ((time-time4)>40 ) {
#      time4=time;
#      // BOTTOM LEFT data display - values
#      accROLL.addVal(ax);
#      accPITCH.addVal(ay);
#      accYAW.addVal(az);
#      gyroROLL.addVal(gx);
#      gyroPITCH.addVal(gy);
#      gyroYAW.addVal(gz);
#      magxData.addVal(magx);magyData.addVal(magy);magzData.addVal(magz);
#      altData.addVal(alt);headData.addVal(head);
#      debug1Data.addVal(debug1);debug2Data.addVal(debug2);debug3Data.addVal(debug3);debug4Data.addVal(debug4);
#    }
#
#    // DATA Request commands....  // USE '/dev/ttyUSB0'......
#    if ((time-time2)>40 && ! toggleRead && ! toggleWrite && ! toggleSetSetting) {
#      time2=time;
#      int[] requests = {MSP_STATUS, MSP_RAW_IMU, MSP_SERVO, MSP_MOTOR, MSP_RC, MSP_RAW_GPS, MSP_COMP_GPS, MSP_ALTITUDE, MSP_BAT, MSP_DEBUGMSG, MSP_DEBUG};
#      sendRequestMSP(requestMSP(requests));
#    }
#    if ((time-time3)>20 && ! toggleRead && ! toggleWrite && ! toggleSetSetting) {
#      sendRequestMSP(requestMSP(MSP_ATTITUDE));
#      time3=time;
#    }
#    if (toggleReset) {
#      toggleReset=false;
#      toggleRead=true;
#      sendRequestMSP(requestMSP(MSP_RESET_CONF));
#    }
#    if (toggleRead) {
#      toggleRead=false;
#      int[] requests = {MSP_BOXNAMES, MSP_RC_TUNING, MSP_PID, MSP_BOX, MSP_MISC, MSP_IDENT, MSP_MOTOR_PINS }; // MSP_PIDNAMES
#      sendRequestMSP(requestMSP(requests));
#      buttonWRITE.setColorBackground(green_);
#      buttonSETTING.setColorBackground(green_);
#      confSelectSetting.setColorBackground(green_);
#    }
#    if (toggleSetSetting) {
#      toggleSetSetting=false;
#      toggleRead=true;
#      payload = new ArrayList<Character>();
#      payload.add(char( round(confSelectSetting.value())) );
#      sendRequestMSP(requestMSP(MSP_SELECT_SETTING,payload.toArray( new Character[payload.size()]) ));
#    }
#    if (toggleCalibAcc) {
#      toggleCalibAcc=false;
#      sendRequestMSP(requestMSP(MSP_ACC_CALIBRATION));
#    }
#    if (toggleCalibMag) {
#      toggleCalibMag=false;
#      sendRequestMSP(requestMSP(MSP_MAG_CALIBRATION));
#    }
#    if (toggleWrite) {
#      toggleWrite=false;
#
#      // MSP_SET_RC_TUNING
#      payload = new ArrayList<Character>();
#      payload.add(char( round(confRC_RATE.value()*100)) );
#      payload.add(char( round(confRC_EXPO.value()*100)) );
#      payload.add(char( round(rollPitchRate.value()*100)) );
#      payload.add(char( round(yawRate.value()*100)) );
#      payload.add(char( round(dynamic_THR_PID.value()*100)) );
#      payload.add(char( round(throttle_MID.value()*100)) );
#      payload.add(char( round(throttle_EXPO.value()*100)) );
#      sendRequestMSP(requestMSP(MSP_SET_RC_TUNING,payload.toArray( new Character[payload.size()]) ));
#
#      // MSP_SET_PID
#      payload = new ArrayList<Character>();
#      for(i=0;i<PIDITEMS;i++) {
#        byteP[i] = (round(confP[i].value()*10));
#        byteI[i] = (round(confI[i].value()*1000));
#        byteD[i] = (round(confD[i].value()));
#      }
#
#      //POS-4 POSR-5 NAVR-6 use different dividers
#      byteP[4] = (round(confP[4].value()*100.0));
#      byteI[4] = (round(confI[4].value()*100.0));
#
#      byteP[5] = (round(confP[5].value()*10.0));
#      byteI[5] = (round(confI[5].value()*100.0));
#      byteD[5] = (round(confD[5].value()*10000.0))/10;
#
#      byteP[6] = (round(confP[6].value()*10.0));
#      byteI[6] = (round(confI[6].value()*100.0));
#      byteD[6] = (round(confD[6].value()*10000.0))/10;
#
#      for(i=0;i<PIDITEMS;i++) {
#          payload.add(char(byteP[i]));
#          payload.add(char(byteI[i]));
#          payload.add(char(byteD[i]));
#      }
#      sendRequestMSP(requestMSP(MSP_SET_PID,payload.toArray(new Character[payload.size()])));
#
#      // MSP_SET_BOX
#      payload = new ArrayList<Character>();
#      for(i=0;i<CHECKBOXITEMS;i++) {
#        activation[i] = 0;
#        for(aa=0;aa<12;aa++) {
#          activation[i] += (int)(checkbox[i].arrayValue()[aa]*(1<<aa));
#          //MWI.setProperty("box."+i+".aux"+i/3+"."+(aa%3),String.valueOf(checkbox[i].arrayValue()[aa]*(1<<aa)));
#        }
#        payload.add(char (activation[i] % 256) );
#        payload.add(char (activation[i] / 256)  );
#      }
#      sendRequestMSP(requestMSP(MSP_SET_BOX,payload.toArray(new Character[payload.size()])));
#
#
#      // MSP_SET_MISC
#      payload = new ArrayList<Character>();
#      intPowerTrigger = (round(confPowerTrigger.value()));
#      payload.add(char(intPowerTrigger % 256));
#      payload.add(char(intPowerTrigger / 256));
#
#      sendRequestMSP(requestMSP(MSP_SET_MISC,payload.toArray(new Character[payload.size()])));
#
#      // MSP_EEPROM_WRITE
#      sendRequestMSP(requestMSP(MSP_EEPROM_WRITE));
#
#      updateModel(); // update model with view value
#    }
#
#    if (toggleRXbind) {
#      toggleRXbind=false;
#      sendRequestMSP(requestMSP(MSP_BIND));
#      bSTOP();
#      InitSerial(9999);
#    }
####################################################

   def read(self):
      c = ord(self.sc.read());
      if self.c_state == self.IDLE:
         self.c_state = self.HEADER_START if c == ord('$') else self.IDLE
      elif self.c_state == self.HEADER_START:
         self.c_state = self.HEADER_M if c == ord('M') else self.IDLE
      elif self.c_state == self.HEADER_M:
         if c == ord('>'):
            self.c_state = self.HEADER_ARROW;
         elif c == ord('!'):
            self.c_state = self.HEADER_ERR;
         else:
            self.c_state = self.IDLE;
      elif self.c_state == self.HEADER_ARROW or self.c_state == self.HEADER_ERR:
         ##/* is this an error message? */
         self.err_rcvd = (self.c_state == self.HEADER_ERR)        ##/* now we are expecting the payload size */
         self.dataSize = (c & 0xFF)
         ##/* reset index variables */
         self.p = 0
         self.offset = 0
         self.checksum = 0
         self.checksum = self.checksum ^ (c & 0xFF)
         ##/* the command is to follow */
         self.c_state = self.HEADER_SIZE
      elif self.c_state == self.HEADER_SIZE:
         self.cmd = (c & 0xFF)
         self.checksum = self.checksum ^ (c & 0xFF)
         self.c_state = self.HEADER_CMD
      elif self.c_state == self.HEADER_CMD and self.offset < self.dataSize:
         self.checksum = self.checksum ^ (c & 0xFF)
         self.inBuf[self.offset] = (byte)(c & 0xFF)
         self.offset += 1
      elif self.c_state == self.HEADER_CMD and self.offset >= self.dataSize:
         ##/* compare calculated and transferred checksum */
         if (self.checksum & 0xFF) == (c & 0xFF):
            if not self.err_rcvd:
               ##/* we got a valid response packet, evaluate it */
               evaluateCommand(self.cmd, int(self.dataSize))
            #else:
            #   #System.err.println("Copter did not understand request type "+c);
         else:
            print "invalid checksum for command "+(int(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+int(c&0xFF)
            print "<"+(cmd&0xFF)+" "+(self.dataSize&0xFF)+"> {"
            for i in range(0, self.dataSize):
               if i != 0:
                  print ' '
               print (self.inBuf[i] & 0xFF)
            print "} ["+c+"]"
            print (self.inBuf[0:self.dataSize]).decode
         c_state = IDLE














#   def setAngle(self, n, angle):
#      if n == 0:
#         if angle < 0:
#            angle=0
#         elif angle > 180:
#            angle=180
#      elif angle > 180 or angle <0:
#         angle=90
#      byteone=int(254*angle/180)
#      bud=chr(0xFF)+chr(n)+chr(byteone)
#      self.sc.write(bud)
#
#   def setPosition(self, servo, position):
#      position = position * 4
#      poslo = (position & 0x7f)
#      poshi = (position >> 7) & 0x7f
#      chan  = servo &0x7f
#      data =  chr(0xaa) + chr(0x0c) + chr(0x04) + chr(chan) + chr(poslo) + chr(poshi)
#      self.sc.write(data)
#
#   def getPosition(self, servo):
#      chan  = servo &0x7f
#      data =  chr(0xaa) + chr(0x0c) + chr(0x10) + chr(chan)
#      self.sc.write(data)
#      w1 = ord(self.sc.read())
#      w2 = ord(self.sc.read())
#      return w1, w2
#
#   def getErrors(self):
#      data =  chr(0xaa) + chr(0x0c) + chr(0x21)
#      self.sc.write(data)
#      w1 = ord(self.sc.read())
#      w2 = ord(self.sc.read())
#      return w1, w2
#
#   def triggerScript(self, subNumber):
#      data =  chr(0xaa) + chr(0x0c) + chr(0x27) + chr(0)
#      self.sc.write(data)
#      w1 = ord(self.sc.read())
#      w2 = ord(self.sc.read())
#      return w1, w2


## MAIN ##

MultiWii = MultiWiiSP()

#time.sleep(2)

# Reading loop. Call it Main??? Make it a gateway function?
#while (g_serial.available()>0) {

#print MultiWii.sc.baudrate
#
#for i in range(0, 5):
#   time.sleep(1)
#   MultiWii.sc.close()
#   print "Port closed... reconnecting...."
#   time.sleep(1)
#   MultiWii.sc.open()
#
#print MultiWii.sc.baudrate

#MultiWii.sendRequestMSP(MultiWii.requestMSP(MultiWii.MSP_IDENT))
#while (MultiWii.sc.inWaiting() <= 0):
#   MultiWii.sc.read()
#   MultiWii.sendRequestMSP(MultiWii.requestMSP(MultiWii.MSP_IDENT))
#   time.sleep(0.5)
#print sys.byteorder

while True:
   requests=[MultiWii.MSP_STATUS, MultiWii.MSP_RAW_IMU, MultiWii.MSP_SERVO, MultiWii.MSP_MOTOR, MultiWii.MSP_RC, MultiWii.MSP_RAW_GPS, MultiWii.MSP_COMP_GPS, MultiWii.MSP_ATTITUDE, MultiWii.MSP_ALTITUDE, MultiWii.MSP_BAT, MultiWii.MSP_DEBUGMSG, MultiWii.MSP_DEBUG ]
   MultiWii.sendRequestMSP(MultiWii.requestsMSP(requests))
   force = True
   #print "Start Process"
   while (MultiWii.sc.inWaiting() > 0 or force == True):
      force = False
      #MultiWii.sc.flush()
      #time.sleep(15)
      #if MultiWii.c_state == MultiWii.IDLE:
      #   print "IDLE"
      #while MultiWii.c_state == MultiWii.IDLE:
      #   #MultiWii.sendRequestMSP(MultiWii.requestMSP(MultiWii.MSP_ATTITUDE))
      #   MultiWii.sendRequestMSP(MultiWii.requestsMSP(requests))
      #   c = MultiWii.sc.read()
      #   #print len(c)
      #   #print MultiWii.sc.read(),
      #   #print "'%s'" % (c,),
      #   if MultiWii.c_state == MultiWii.IDLE:
      #      MultiWii.c_state = MultiWii.HEADER_START if c == '$' else MultiWii.IDLE
      #      #print "IDLE -> ","HEADER_START" if c == '$' else "IDLE"
      #
      ##c = ord((MultiWii.sc.read())[0])
      #MultiWii.sendRequestMSP(MultiWii.requestsMSP(requests))
      c = MultiWii.sc.read()
      #print c,
      if MultiWii.c_state == MultiWii.IDLE:
         MultiWii.c_state = MultiWii.HEADER_START if c == '$' else MultiWii.IDLE
      elif MultiWii.c_state == MultiWii.HEADER_START:
         #print "HEADER_START"
         MultiWii.c_state = MultiWii.HEADER_M if c == 'M' else MultiWii.IDLE
      elif MultiWii.c_state == MultiWii.HEADER_M:
         #print "HEADER_M"
         if c == '>':
            MultiWii.c_state = MultiWii.HEADER_ARROW;
         elif c == '!':
            MultiWii.c_state = MultiWii.HEADER_ERR;
         else:
            MultiWii.c_state = MultiWii.IDLE;
      elif MultiWii.c_state == MultiWii.HEADER_ARROW or MultiWii.c_state == MultiWii.HEADER_ERR:
         ##/* is this an error message? */
         MultiWii.err_rcvd = (MultiWii.c_state == MultiWii.HEADER_ERR)        ##/* now we are expecting the payload size */
         #print "c len, value: ",len(c),", ",c
         #print "c len: %d, value: '%s'" % (len(c),c,),
         MultiWii.dataSize = (ord(c) & 0xFF)
         ##/* reset index variables */
         MultiWii.p = 0
         MultiWii.offset = 0
         MultiWii.checksum = 0
         MultiWii.checksum = MultiWii.checksum ^ (ord(c) & 0xFF)
         ##/* the command is to follow */
         MultiWii.c_state = MultiWii.HEADER_SIZE
      elif MultiWii.c_state == MultiWii.HEADER_SIZE:
         MultiWii.cmd = (ord(c) & 0xFF)
         MultiWii.checksum = MultiWii.checksum ^ (ord(c) & 0xFF)
         MultiWii.c_state = MultiWii.HEADER_CMD
      elif MultiWii.c_state == MultiWii.HEADER_CMD and MultiWii.offset < MultiWii.dataSize:
         MultiWii.checksum = MultiWii.checksum ^ (ord(c) & 0xFF)
         #print "offset",MultiWii.offset
         if len(MultiWii.inBuf) <= MultiWii.offset:
            MultiWii.inBuf.append(ord(c) & 0xFF)
         else:
            MultiWii.inBuf[MultiWii.offset] = (ord(c) & 0xFF)
         MultiWii.offset += 1
      elif MultiWii.c_state == MultiWii.HEADER_CMD and MultiWii.offset >= MultiWii.dataSize:
         ##/* compare calculated and transferred checksum */
         if (MultiWii.checksum & 0xFF) == (ord(c) & 0xFF):
            if not MultiWii.err_rcvd:
               ##/* we got a valid response packet, evaluate it */
               MultiWii.evaluateCommand(MultiWii.cmd, int(MultiWii.dataSize))
            #else:
            #   #System.err.println("Copter did not understand request type "+c);
         else:
            print "invalid checksum for command ",(int(cmd&0xFF)),": ",(checksum&0xFF)," expected, got ",int(ord(c)&0xFF)
            print "<",(cmd&0xFF)," ",(MultiWii.dataSize&0xFF),"> {"
            for i in range(0, MultiWii.dataSize):
               if i != 0:
                  print ' '
               print (MultiWii.inBuf[i] & 0xFF)
            print "} [",c,"]"
            print (MultiWii.inBuf[0:MultiWii.dataSize]).decode
         MultiWii.c_state = MultiWii.IDLE
   
   #print "Heading:",MultiWii.head, "x-angle:", MultiWii.angx, "y-angle:", MultiWii.angy
   #print "Acc: x:",MultiWii.ax, "y:",MultiWii.ay,"z:",MultiWii.az
   #print "Gyro: x:",MultiWii.gx, "y:",MultiWii.gy,"z:",MultiWii.gz
   #print "Mag: x:",MultiWii.magx, "y:",MultiWii.magy,"z:",MultiWii.magz
   print "RC: Throt:",MultiWii.rcThrottle, "Yaw:",MultiWii.rcYaw, "Pitch:",MultiWii.rcPitch, "Roll:",MultiWii.rcRoll, "AUX1:",MultiWii.rcAUX1, "AUX2:",MultiWii.rcAUX2 #,"AUX3:",MultiWii.rcAUX3,"AUX4:",MultiWii.rcAUX4
   #print "Alt:",MultiWii.alt
   #print "Motors: 1:",  MultiWii.mot[0], "2:",  MultiWii.mot[1], "3:",  MultiWii.mot[2], "4:",  MultiWii.mot[3], "5:",  MultiWii.mot[4], "6:",  MultiWii.mot[5], "7:",  MultiWii.mot[6], "8:",  MultiWii.mot[7]  
   #print "Servos: 1:",MultiWii.servo[0], "2:",MultiWii.servo[1], "3:",MultiWii.servo[2], "4:",MultiWii.servo[3], "5:",MultiWii.servo[4], "6:",MultiWii.servo[5], "7:",MultiWii.servo[6], "8:",MultiWii.servo[7]
   
   #thro = int(raw_input("Throttle: ")) * 100
   #yaw  = int(raw_input("Yaw     : ")) * 100
   #pitc = int(raw_input("Pitch   : ")) * 100
   #roll = int(raw_input("Roll    : ")) * 100
   #aux1 = 1500 #int(raw_input("AUX1    : ")) * 100
   #aux2 = int(raw_input("AUX2    : ")) * 100
   #aux3 = 1500 #int(raw_input("AUX3    : ")) * 100
   #aux4 = 1500 #int(raw_input("AUX4    : ")) * 100
   
   readData = None
   with open('RC_pos.txt', 'r') as fin:
      readData = fin.read()
   fin.close()
   
   #print "'",readData[4],"'" # 4,9,14,19,24,29,34,39
   thro = int(readData[0:4])
   yaw  = int(readData[5:9])
   pitc = int(readData[10:14])
   roll = int(readData[15:19])
   aux1 = int(readData[20:24])
   aux2 = int(readData[25:29])
   aux3 = int(readData[30:34])
   aux4 = int(readData[35:39])
   
   str1  = MultiWii.format16.pack(roll)
   str1 += MultiWii.format16.pack(pitc)
   str1 += MultiWii.format16.pack(yaw )
   str1 += MultiWii.format16.pack(thro)
   str1 += MultiWii.format16.pack(aux1)
   str1 += MultiWii.format16.pack(aux2)
   str1 += MultiWii.format16.pack(aux3)
   str1 += MultiWii.format16.pack(aux4)
   
   MultiWii.sendRequestMSP(MultiWii.requestFormatMSP_WP(MultiWii.MSP_SET_RAW_RC, str1))
   #MultiWii.read()
   #time.sleep(1)
del MultiWii


#
#iterations = 2
#
#while(iterations > 0):
#    for chan in range(0,3):
#        for pos in range(0, 180):
#            Maestro.setAngle(chan, pos)
#            time.sleep(.002)
#            #print Maestro.getPosition(chan), Maestro.getErrors()
#
#    for chan in range(0,3):
#        for pos in range(180, 0, -1):
#            Maestro.setAngle(chan, pos)
#            time.sleep(.002)
#            #print Maestro.getPosition(chan)
#
#    e1, e2 = Maestro.getErrors()
#    if e1 != 0 or e2 != 0:
#       print e1, e2
#       break
#
#    iterations = iterations - 1
#
#    print "Trigger Script"
#    Maestro.triggerScript(0)
#
#time.sleep(2)
#Maestro.closeServo()