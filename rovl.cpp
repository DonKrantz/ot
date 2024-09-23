#include "rovl.h"
#include "utilities.h"
#include "system_state.h"
#include "gnss_ping_protocol.h"

// ======================================================================
void rovl_reset_status()
{
   rovl_valid_rx = false;
   configured_correctly_rx = false;
}


// ======================================================================
void decode_poll(string s)
{
   rovl_polling_ids_human_readable = head_of(s, "*", true);

   s = rovl_polling_ids_human_readable;
   uint16_t mask = 0;

   while (s != "")
   {
      string n = head_of(s, " ", true);

      int shift = 0;
      if (n != "")
         shift = std::stoi(n);
      else
         continue;

      mask |= (0b1 << shift);
   }

   rovl_polling_ids_mask = mask;
}


// ======================================================================
void parse_usrth(string s)
{
   rovl_usrth.ping_group_valid = true;
   rovl_usrth.imu_group_valid = true;

   string dummys;
   bool dummyb;

   parseNMEA(s, "sfffffffffffississss",

      &dummys, &dummyb, // the message ID

      & rovl_usrth.apparent_bearing_math, &rovl_usrth.ping_group_valid,
      & rovl_usrth.apparent_bearing_compass, &rovl_usrth.ping_group_valid,
      & rovl_usrth.apparent_elevation, &rovl_usrth.ping_group_valid,
      & rovl_usrth.slant_range, &rovl_usrth.ping_group_valid,
      & rovl_usrth.true_bearing_math, &rovl_usrth.ping_group_valid,
      & rovl_usrth.true_bearing_compass, &rovl_usrth.ping_group_valid,
      & rovl_usrth.true_elevation, &rovl_usrth.ping_group_valid,

      & rovl_usrth.euler_roll, &rovl_usrth.imu_group_valid,
      & rovl_usrth.euler_pitch, &rovl_usrth.imu_group_valid,
      & rovl_usrth.euler_yaw, &rovl_usrth.imu_group_valid,
      & rovl_usrth.heading, &rovl_usrth.imu_group_valid,

      & rovl_usrth.adc_gain, &rovl_usrth.dummy,
      & rovl_usrth.autosync_supported, &rovl_usrth.dummy,
      & rovl_usrth.autosync_active, &rovl_usrth.dummy,
      & rovl_usrth.seconds_since_sync, &rovl_usrth.dummy,
      & rovl_usrth.imu_status, &rovl_usrth.dummy,
      & rovl_usrth.channel, &rovl_usrth.dummy,
      & rovl_usrth.id_decoded, &rovl_usrth.dummy,
      & rovl_usrth.id_queried, &rovl_usrth.dummy
   );

   // TODO: Request Gnss orientation/location 
   send_ping_request(imu_gnss_compass_data);

}

// ======================================================================
float get_hex_float(string s, int& ptr)
{
   string hexdigit = "0123456789ABCDEF";

   uint32_t val = 0;

   for (int i = 0; i < 8; i++)
   {
      val <<= 4;
      int dig = (hexdigit.find_first_of(s[ptr++]) & 0xF);
      val |= dig;
   }

   return *((float *) (&val));
}

// ======================================================================
// This is a packed Hex-encoded representation of 9 floats and one bool
void parse_usimx(string s)
{
   struct usimu_message msg;
   UNUSED(msg);

   int ptr = 7;   // point past the message ID

   msg.delta_t = get_hex_float(s, ptr);
   msg.accel.x = get_hex_float(s, ptr);
   msg.accel.y = get_hex_float(s, ptr);
   msg.accel.z = get_hex_float(s, ptr);
   msg.mag.x = get_hex_float(s, ptr);
   msg.mag.y = get_hex_float(s, ptr);
   msg.mag.z = get_hex_float(s, ptr);
   msg.gyro.x = get_hex_float(s, ptr);
   msg.gyro.y = get_hex_float(s, ptr);
   msg.gyro.z = get_hex_float(s, ptr);
   msg.is_pinging = s[ptr] == '1';
}

// ======================================================================
void parse_rovlrx(string s, double timestamp)
{
   static int fails = 0;

   if ( (s[0] == '$') && (!NMEA_checksum_valid(s)) )
   {
      fails++;

      log_warning("%1.4f ROVL-RX NMEA checksum fails = %d", timestamp, fails);
      log_warning("Offending message: %s\n", s.c_str());

 //     for (char c : s)
 //        printf("%02X ", c);
 //     printf("\n");
   }
   else if (s[0] != '$')
   {
      log_warning("%1.4f ROVL-RX: %s\n", timestamp, s.c_str());
   }

   if (contains("$USRTH", s))
   {
      parse_usrth(s);
      omnifusion.fuseRovl(rovl_usrth.apparent_bearing_math, rovl_usrth.apparent_elevation, rovl_usrth.slant_range);
      // todo: this is where the serializer delivers ROVL $USRTH messages

   }
   
   else if (contains("$USIMX", s))
   {
      parse_usimx(s);
   }
   
   else
   {
      string temp = "";
      if ((temp = free_split("Firmware version ", s)) != "")
         rovl_firmware_rx = temp;
      if ((temp = free_split("Magnetic declination = ", s)) != "")
         rovl_magnetic_declination = std::stof(temp);
      if ((temp = free_split("CIMU is ", s)) != "")
         rovl_using_CIMU = true;
      if ((temp = free_split("Speed of sound = ", s)) != "")
         rovl_speed_of_sound = std::stof(temp);
      if ((temp = free_split("No IDs selected", s)) != "")
      {
         rovl_polling_ids_mask = 0; rovl_valid_rx = true;
      }
      if ((temp = free_split("Polling IDs ", s)) != "")
      {
         decode_poll(temp); rovl_valid_rx = true;
      }
   }
}


