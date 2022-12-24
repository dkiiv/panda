def create_pq_acc_control(packer, bus, idx, enabled, accel_req, stopping, acc_type):
  values = {
    "ACS_Zaehler": idx,                             //# ADR Counter
    "ACS_Sta_ADR": 1 if enabled else 2,             //# ADR Status (0 inactive / 1 active / 2 passive / 3 irreversible fault)
    "ACS_StSt_Info": 1,                             //# StartStop request (0 Allow stop / 1 Engine start not needed / 2 Engine start / 3 failure)
    "ACS_MomEingriff": 0,                           //# Torque intervention (Prevent Whiplash?) (0 Allow Whiplash / 1 Don't allow whiplash)
    "ACS_Typ_ACC": acc_type,                        //# ADR Type (0 Basic ACC / 1 ACC Follow2Stop / 2 unused / 3 ACC not installed)
    "ACS_FreigSollB": 1 if enabled else 0,          //# Activation of ACS_Sollbeschl (0 not allowed / 1 allowed)
    "ACS_Sollbeschl": accel_req if enabled else 10.23,    //# Acceleration Request (2046(10.23) ADR Inactive / 2047(10.235) Fault)
    "ACS_Anhaltewunsch": 1 if stopping else 0,      //# Stopping Request (0 No stop request / 1 Vehicle stopping)
    "ACS_zul_Regelabw": 0.4 if enabled else 1.27,     //# Allowed request deviation (254 ADR not active / 255 Fault) (Use this offset if more comfort can be achieved)
    "ACS_max_AendGrad": 1 if enabled else 0,      //# Allowed gradient changes (0 Neutral value, 254 Neutral value, 255 Fault) (Unknown)
  }

  dat = packer.make_can_msg("ACC_System", bus, values)[2]
  values["ACS_Checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("ACC_System", bus, values)

def create_pq_aca_control(packer, bus , idx, enabled, set_speed, metric, acc_coding):
  values = {
    "ACA_StaACC": 3 if enabled else 2,         //# ADR Status in cluster (0 Switch Off / 2 ACC Pasive / 3 ACC Active / 4 ACC in Background / 6 ACC reversible off / 7 ACC irreversible off)
    "ACA_Fahrerhinw": 0,                      //# ADR Driver Warning (Max Limit reached) (0 Off / 1 On)
    "ACA_AnzDisplay": 1 if enabled else 0,     //# ADR Display Status (0 No Display / 1 Display)
    "ACA_Zeitluecke": 1,                      //# Display set time gap (0 Not defined / 1-15 different distances for display in cluster)
    "ACA_V_Wunsch": set_speed,                //# Display set speed (255 Not set (yet) / 0-254 Actual KM/h setpoint)
    "ACA_kmh_mph": 0 if metric else 1,        //# Display KMh or Mph (0 Km/h / 1 Mph)
    "ACA_Akustik1": 0,                        //# Soft cluster gong (0 Off / 1 On)
    "ACA_Akustik2": 0,                        //# Hard cluster buzzer (0 Off / 1 On)
    "ACA_PrioDisp": 1,                        //# Display Priority (0 High Prio / 1 Prio / 2 Low Prio / 3 No Request)
    "ACA_gemZeitl": 0,                        //# Avarage Follow distance (0 No lead / 1-15 Actual avarge distance)
    "ACA_Codierung": acc_coding,                       //# Coding (0 ACC / 1 GRA)
    "ACA_Zaehler": idx,                       //# Counter
  }

  dat = packer.make_can_msg("ACC_GRA_Anzeige", bus, values)[2]
  values["ACA_Checksum"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6] ^ dat[7]
  return packer.make_can_msg("ACC_GRA_Anzeige", bus, values)