// Writtwn by: Emmard Frangopoulos
// Date: 01.04.2025

// #include "navigation.h"
// #include <iostream>
// #include <vector>

// #include <fstream>  // For writing the data to a file
// #include <sstream>
// #include <string>


// using namespace std;


// // struct RawData {
// //     int timestamp;
// //     double gyroX1, gyroY1, gyroZ1;
// //     double accelX1, accelY1, accelZ1;
// //     double gyroX2, gyroY2, gyroZ2;
// //     double accelX2, accelY2, accelZ2;
// //     double gyroX3, gyroY3, gyroZ3;
// //     double accelX3, accelY3, accelZ3;
// //     double pressure1, pressure2, pressure3;
// //     double gpsLat, gpsLon, gpsAlt;
// // };


// std::vector<RawData> readCSV(const std::string &filename);

// int main() {

//     std::string filename = "C:/Users/zorba/Documents/EPFL/ERT/NAVIGATION/simpleNav/simple_nav_c++_v1/test/still_data_04.12.24.csv";
//     //std::string filename = "C:/Users/zorba/Documents/EPFL/ERT/NAVIGATION/simpleNav/simple_nav_c++/test/straight_line_data_04.12.24.csv";
//     //std::string filename = "C:/Users/zorba/Documents/EPFL/ERT/NAVIGATION/simpleNav/simple_nav_c++/test/circle_data_04.12.24.csv";
    
//     std::vector<RawData> rawData = readCSV(filename);

//     double time = rawData[0].timestamp;
//     double gps_latitude = rawData[0].gpsLat, gps_longitude = rawData[0].gpsLon, gps_altitude = rawData[0].gpsAlt;
//     double baro1 = rawData[0].pressure1, baro2 = rawData[0].pressure2, baro3 = rawData[0].pressure3;
//     double temperature = 288.15;
//     double gyroX1 = rawData[0].gyroX1, gyroY1 = rawData[0].gyroY1, gyroZ1 = rawData[0].gyroZ1;
//     double gyroX2 = rawData[0].gyroX2, gyroY2 = rawData[0].gyroY2, gyroZ2 = rawData[0].gyroZ2;
//     double gyroX3 = rawData[0].gyroX3, gyroY3 = rawData[0].gyroY3, gyroZ3 = rawData[0].gyroZ3;


//     Navigation nav(time, baro1, baro2, baro3, temperature, gps_latitude, gps_longitude, gps_altitude, gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2, gyroX3, gyroY3, gyroZ3);


//     // Vectors to store data for plotting
//     vector<double> time_data;
//     vector<double> position_data_x, position_data_y, position_data_z;
//     vector<double> velocity_data_x, velocity_data_y, velocity_data_z;
//     vector<double> acceleration_data_x, acceleration_data_y, acceleration_data_z;
//     vector<double> orientation_data_roll, orientation_data_pitch, orientation_data_yaw;
//     vector<double> angular_velocity_data_x, angular_velocity_data_y, angular_velocity_data_z;

//     // Store the data at the initial time
//     time_data.push_back(time);
//     position_data_x.push_back(0);
//     position_data_y.push_back(0);
//     position_data_z.push_back(0);
//     velocity_data_x.push_back(0);
//     velocity_data_y.push_back(0);
//     velocity_data_z.push_back(0);
//     acceleration_data_x.push_back(0);
//     acceleration_data_y.push_back(0);
//     acceleration_data_z.push_back(0);
//     orientation_data_roll.push_back(0);
//     orientation_data_pitch.push_back(0);
//     orientation_data_yaw.push_back(0);
//     angular_velocity_data_x.push_back(0);
//     angular_velocity_data_y.push_back(0);
//     angular_velocity_data_z.push_back(0);
    
//     for (const auto &data : rawData)
//     {
        
//         time = data.timestamp;
//         gps_latitude = data.gpsLat, gps_longitude = data.gpsLon, gps_altitude = data.gpsAlt;
//         baro1 = data.pressure1, baro2 = data.pressure2, baro3 = data.pressure3;
//         temperature = 288.15;
//         gyroX1 = data.gyroX1, gyroY1 = data.gyroY1, gyroZ1 = data.gyroZ1;
//         gyroX2 = data.gyroX2, gyroY2 = data.gyroY2, gyroZ2 = data.gyroZ2;
//         gyroX3 = data.gyroX3, gyroY3 = data.gyroY3, gyroZ3 = data.gyroZ3;

//         nav.update(time, baro1, baro2, baro3, temperature, gps_latitude, gps_longitude, gps_altitude, gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2, gyroX3, gyroY3, gyroZ3);

//         vector<double> state = nav.get_state();
        
//         /*
//         cout << "State: " << endl;
//         cout << "Position : " << state[0] << " " << state[1] << " " << state[2] << endl;
//         cout << "Velocity : " << state[3] << " " << state[4] << " " << state[5] << endl;
//         cout << "Acceleration : " << state[6] << " " << state[7] << " " << state[8] << endl;
//         cout << "Orientation : " << state[9] << " " << state[10] << " " << state[11] << endl;
//         cout << "Angular Velocity : " << state[12] << " " << state[13] << " " << state[14] << endl << endl;
//         */

//         // Store the data
//         time_data.push_back(time);
//         position_data_x.push_back(state[0]);
//         position_data_y.push_back(state[1]);
//         position_data_z.push_back(state[2]);
//         velocity_data_x.push_back(state[3]);
//         velocity_data_y.push_back(state[4]);
//         velocity_data_z.push_back(state[5]);
//         acceleration_data_x.push_back(state[6]);
//         acceleration_data_y.push_back(state[7]);
//         acceleration_data_z.push_back(state[8]);
//         orientation_data_roll.push_back(state[9]);
//         orientation_data_pitch.push_back(state[10]);
//         orientation_data_yaw.push_back(state[11]);
//         angular_velocity_data_x.push_back(state[12]);
//         angular_velocity_data_y.push_back(state[13]);
//         angular_velocity_data_z.push_back(state[14]);
//     }


//     // Write the data to a file (for use with Python plotting)
//     ofstream data_file("state_data.csv");
//     data_file << "Time,PosX,PosY,PosZ,VelX,VelY,VelZ,AccX,AccY,AccZ,OriRoll,OriPitch,OriYaw,AngVelX,AngVelY,AngVelZ\n";

//     for (size_t i = 0; i < time_data.size(); ++i) {
//         data_file << time_data[i] << "," 
//                   << position_data_x[i] << "," << position_data_y[i] << "," << position_data_z[i] << ","
//                   << velocity_data_x[i] << "," << velocity_data_y[i] << "," << velocity_data_z[i] << ","
//                   << acceleration_data_x[i] << "," << acceleration_data_y[i] << "," << acceleration_data_z[i] << ","
//                   << orientation_data_roll[i] << "," << orientation_data_pitch[i] << "," << orientation_data_yaw[i] << ","
//                   << angular_velocity_data_x[i] << "," << angular_velocity_data_y[i] << "," << angular_velocity_data_z[i] << "\n";
//     }

//     data_file.close();

//     return 0;

// }



// // Function to read the CSV file and parse it into GPSData objects
// std::vector<RawData> readCSV(const std::string &filename)
// {
//     std::vector<RawData> rawDataList;
//     std::ifstream file(filename);
//     std::string line;
    
//     // Skip header line if there is one
//     // std::getline(file, line); 

//     if (!file.is_open()) {
//         std::cerr << "Erreur lors de l'ouverture du fichier " << filename << std::endl;
//         return rawDataList;
//     }

//     // Pas d'ignorer la première ligne, puisque c'est une ligne de données
//     std::cout << "Lecture des données du fichier..." << std::endl;


//     while (std::getline(file, line)) {
//         std::istringstream ss(line);
//         RawData data;

//         // Lire les données séparées par des virgules
//         char comma;  // pour ignorer les virgules
//         if (ss >> data.timestamp >> comma >> data.gyroX1 >> comma >> data.gyroY1 >> comma >> data.gyroZ1 >> comma
//                 >> data.accelX1 >> comma >> data.accelY1 >> comma >> data.accelZ1 >> comma >> data.gyroX2 >> comma
//                 >> data.gyroY2 >> comma >> data.gyroZ2 >> comma >> data.accelX2 >> comma >> data.accelY2 >> comma
//                 >> data.accelZ2 >> comma >> data.gyroX3 >> comma >> data.gyroY3 >> comma >> data.gyroZ3 >> comma
//                 >> data.accelX3 >> comma >> data.accelY3 >> comma >> data.accelZ3 >> comma >> data.pressure1 >> comma
//                 >> data.pressure2 >> comma >> data.pressure3 >> comma >> data.gpsLat >> comma >> data.gpsLon >> comma
//                 >> data.gpsAlt) {
//             rawDataList.push_back(data);
//         }
//     }

//     file.close();

//     // Vérifier si des données ont été lues
//     if (rawDataList.empty()) {
//         std::cerr << "Aucune donnée lue depuis le fichier." << std::endl;
//     }

//     return rawDataList;
// }