
#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0

#define TUNINGS_FOLDER "tunings/"
#define TUNING_FILE_NAME "drone_config.json"

// Stream& output;

DynamicJsonDocument setup_json(){
  File jsonFile;

  if(!SD.exists(TUNING_FILE_NAME)){
    Serial.print(TUNING_FILE_NAME);
    Serial.println(" tuning file does not exists");
  }
  jsonFile = SD.open(TUNING_FILE_NAME, FILE_READ);  // Assume the file name is "drone_config.json"

  if (!jsonFile) {
    Serial.println("Failed to open json file for reading.");
    while(1){}
  }

  Serial.println("Reading tuning");
  // Create a DynamicJsonDocument to hold the deserialized data
  DynamicJsonDocument doc(1024); // Adjust size as needed

  // Deserialize the JSON data from the file
  DeserializationError error = deserializeJson(doc, jsonFile);

  if(error){
    Serial.println("Failed to desirialize json tuning");
    while(1){}
  }

  jsonFile.close();  // Close the file after reading
  return doc;
}


void write_json(JsonDocument doc, FlightNumber number){
  std::string log_name = TUNINGS_FOLDER + std::to_string(number.session_number) + "_" + std::to_string(number.flight_number) + ".json";

  File jsonFile = SD.open(log_name.c_str(), FILE_WRITE);
  if (!jsonFile) {
    Serial.print(log_name.c_str());
    Serial.println(" Failed to open json file for writting");
    while(1){}
  }

  serializeJsonPretty(doc, jsonFile);
  serializeJsonPretty(doc, Serial);

  jsonFile.close();
}

// JsonDocument serializeDrone(Controller cont){

//     JsonDocument doc;

//     JsonObject position = doc["position"].to<JsonObject>();

//     JsonArray position_p = position["p"].to<JsonArray>();
//     for(int i = 0; i < 3; i++){
//         position_p.add(cont.posCont.pGain[i]);
//     }

//     JsonArray position_i = position["i"].to<JsonArray>();
//     for(int i = 0; i < 3; i++){
//         position_i.add(cont.posCont.iGain[i]);
//     }

//     JsonArray position_d = position["d"].to<JsonArray>();
//     for(int i = 0; i < 3; i++){
//         position_d.add(cont.posCont.dGain[i]);
//     }

//     JsonObject velocity = doc["velocity"].to<JsonObject>();

//     JsonArray velocity_p = velocity["p"].to<JsonArray>();
//     velocity_p.add(0);
//     velocity_p.add(0);
//     velocity_p.add(0);

//     JsonArray velocity_i = velocity["i"].to<JsonArray>();
//     velocity_i.add(0);
//     velocity_i.add(0);
//     velocity_i.add(0);

//     JsonArray velocity_d = velocity["d"].to<JsonArray>();
//     velocity_d.add(0);
//     velocity_d.add(0);
//     velocity_d.add(0);

//     JsonObject attitude = doc["attitude"].to<JsonObject>();

//     JsonArray attitude_p = attitude["p"].to<JsonArray>();
//     attitude_p.add(0);
//     attitude_p.add(0);
//     attitude_p.add(0);

//     JsonArray attitude_i = attitude["i"].to<JsonArray>();
//     attitude_i.add(0);
//     attitude_i.add(0);
//     attitude_i.add(0);

//     JsonArray attitude_d = attitude["d"].to<JsonArray>();
//     attitude_d.add(0);
//     attitude_d.add(0);
//     attitude_d.add(0);

//     JsonObject rate = doc["rate"].to<JsonObject>();

//     JsonArray rate_p = rate["p"].to<JsonArray>();
//     rate_p.add(0);
//     rate_p.add(0);
//     rate_p.add(0);

//     JsonArray rate_i = rate["i"].to<JsonArray>();
//     rate_i.add(0);
//     rate_i.add(0);
//     rate_i.add(0);

//     JsonArray rate_d = rate["d"].to<JsonArray>();
//     rate_d.add(0);
//     rate_d.add(0);
//     rate_d.add(0);

//     doc.shrinkToFit();  // optional

//     return doc;
//     //serializeJson(doc, output);
// }

// Controller Controller_fromJson(DynamicJsonDocument doc){
//   Vec3 P = {0, 0, 0};
//   Vec3 I = {0, 0, 0};
//   Vec3 D = {0, 0, 0};


//   JsonObject position = doc["position"];
//   JsonArray position_p = position["p"];
//   JsonArray position_i = position["i"];
//   JsonArray position_d = position["d"];

//   JsonObject velocity = doc["velocity"];
//   JsonArray velocity_p = velocity["p"];
//   JsonArray velocity_i = velocity["i"];
//   JsonArray velocity_d = velocity["d"];

//   JsonObject attitude = doc["attitude"];
//   JsonArray attitude_p = attitude["p"];
//   JsonArray attitude_i = attitude["i"];
//   JsonArray attitude_d = attitude["d"];

//   JsonObject rate = doc["rate"];
//   JsonArray rate_p = rate["p"];
//   JsonArray rate_i = rate["i"];
//   JsonArray rate_d = rate["d"];
  

//   for (int i = 0; i < 3; i++) {
//     P[i] = position_p[i].as<double>();
//     I[i] = position_i[i].as<double>();
//     D[i] = position_d[i].as<double>();
//   }

//   const PID posControl = {P, I, D, Controller::POS_DT, SPEED_LIMIT};

//     for (int i = 0; i < 3; i++) {
//     P[i] = velocity_p[i].as<double>();
//     I[i] = velocity_i[i].as<double>();
//     D[i] = velocity_d[i].as<double>();
//   }

//   const PID velControl = {P, I, D, Controller::VEL_DT, ACC_LIMIT};
  
//       for (int i = 0; i < 3; i++) {
//     P[i] = attitude_p[i].as<double>();
//     I[i] = attitude_i[i].as<double>();
//     D[i] = attitude_d[i].as<double>();
//   }
//   const PID attControl = {P, I, D, Controller::ATT_DT, ANG_RATE_LIMIT};

//     for (int i = 0; i < 3; i++) {
//     P[i] = rate_p[i].as<double>();
//     I[i] = rate_i[i].as<double>();
//     D[i] = rate_d[i].as<double>();
//   }
//   const PID rateControl = {P, I, D, Controller::RATE_DT, {TORQUE_LIMIT_XY, TORQUE_LIMIT_XY, TORQUE_LIMIT_Z}, {-0.0, 0.0, 0.}};
//   return {rateControl, attControl, velControl, posControl,
//                                      M, R, GIMBAL_LIMIT, GIMBAL_RATE_LIMIT,
//                                      MAX_THRUST,THRUST_RATE_LIMIT};
  

// }