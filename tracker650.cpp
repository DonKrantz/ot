#include "tracker650.h"
#include "utilities.h"
#include "system_state.h"


// ======================================================================
void t650_reset_status()
{
   t650_valid = false;
}


// ======================================================================
void parse_dvpdx(string s, struct dvpdx_message dvpdx)
{
   string dummys;
   bool dummyb;

   dvpdx.angle_delta_group_valid = true;
   dvpdx.position_group_valid = true;

   parseNMEA(s, "siiffffffiifff",

      &dummys, &dummyb, // the message ID

      & dvpdx.time_uS, &dummyb,
      & dvpdx.delta_time_uS, &dummyb,

      & dvpdx.angle_delta_roll, & dvpdx.angle_delta_group_valid,
      & dvpdx.angle_delta_pitch, & dvpdx.angle_delta_group_valid,
      & dvpdx.angle_delta_yaw, & dvpdx.angle_delta_group_valid,

      & dvpdx.position_delta_x, & dvpdx.position_group_valid,
      & dvpdx.position_delta_y, & dvpdx.position_group_valid,
      & dvpdx.position_delta_z, & dvpdx.position_group_valid,

      & dvpdx.confidence, & dummyb,
      & dvpdx.mode, & dummyb,

      & dvpdx.pitch, & dummyb,
      & dvpdx.roll, & dummyb,
      & dvpdx.standoff, & dummyb
   );

}


// ======================================================================
void parse_t650(string s, double timestamp)
{
   static int fails = 0;

   if ((s[0] == '$') && (!NMEA_checksum_valid(s)))
   {
      fails++;

      log_warning("%1.4f t650 NMEA checksum fails = %d", timestamp, fails);
      log_warning("Offending message: %s", s.c_str());
   }
   else if (s[0] != '$')
   {
      log_warning("%1.4f Tracker650: %s", timestamp, s.c_str());
//      for (char c : s)
//         printf("%02X ", c);
//      printf("\n");
   }
   
   if (contains("$DVPDX", s))
   {
      parse_dvpdx(s, t650_dvpdx);
      // todo: this is where the serializer delivers Tracker650 DVPDX messages
   }

   else if (contains("$DVNVM", s))
   {
      t650_dvnvm = s;
      t650_valid = true;
   }

   else
   {
      string temp = "";
      log_event("Tracker650 message: %s", s.c_str());
#if false
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
#endif
   }
}

