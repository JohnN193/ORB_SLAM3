/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <boost/filesystem.hpp>
#include<System.h>

using namespace std;
using namespace boost::filesystem;
// void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                 vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
double readTimeFromFilename(string filename);
void LoadImagesNew(const string &path_to_data,
                vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimeStamps);
                void LoadImages(const string &strPathRGB, const string &strPathD, const string &strPathTimes,
                vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimeStamps);
std::vector<std::string> listFilesInDirectoryForCamera(
    std::string data_directory, std::string extension,
    std::string camera_name);
int main(int argc, char **argv)
{
    
    if(argc < 5)
    {
        cerr << endl << "Usage: ./rgbd_file path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (trajectory_file_name)" << endl;
	 cerr << endl << "Example: ./initialAttempt/rgbd_file ./Vocabulary/ORBvoc.txt ./initialAttempt/realsense515_depth.yaml ./officePics Out_file.txt outputPose" << endl;
        return 1;
    }
    string file_name,file_nameTraj,file_nameKey;


        file_name = string(argv[argc - 1]);
        file_nameTraj = file_name;
        file_nameKey = file_name;
        file_nameTraj = file_nameTraj.append(".txt");
        file_nameKey = file_nameKey.append("Keyframe.txt");


    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3]) + "/" + string(argv[4]);
     string pathSeq(argv[3]);
//     LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
 	
	string pathCam0 = pathSeq + "/rgb";
	string pathCam1 = pathSeq + "/depth";
    LoadImages(pathCam0, pathCam1, strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    // LoadImagesNew(pathSeq, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true, 0, file_nameTraj);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    Sophus::SE3f pose;
     std::chrono::steady_clock::time_point timeStart = std::chrono::steady_clock::now();
    double tStart = vTimestamps[0];
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];
        cout << tframe << "  color:" << vstrImageFilenamesRGB[ni] << endl;

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }
        cout << "yo" << endl;
        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);
        // pose = SLAM.TrackRGBD(imRGB,imD,tframe);
        // cout << pose.translation() << endl;
        // cout << "yo" << endl;
        // cout << pose.log() << endl;
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    std::chrono::steady_clock::time_point timeEnd = std::chrono::steady_clock::now();
    double timeSLAM = std::chrono::duration_cast<std::chrono::duration<double> >(timeEnd - timeStart).count();
    cout <<"Total time for SLAM: " << timeSLAM << endl;
    // Stop all threads
    std::vector<ORB_SLAM3::MapPoint*> mapStuff = SLAM.GetAtlas()->GetCurrentMap()->GetAllMapPoints();
        // Map* GetCurrentMap();
        // mapStuff = SLAM.GetTrackedMapPoints();
        cout << "Start to write PCD with datapoints: " << endl;
        cout << mapStuff.size() << endl;
        // std::cout << "# x,y,z" << std::endl;
        string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(file_name);
        pathSaveFileName.append(".pcd");
        std::remove(pathSaveFileName.c_str());
        std::ofstream ofs(pathSaveFileName, std::ios::binary);
        // boost::archive::text_oarchive oa(ofs);
        ofs  << "VERSION .7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4\n"
            << "TYPE F F F\n"
            << "COUNT 1 1 1\n"
            << "WIDTH "
            << mapStuff.size()
            << "\n"
            << "HEIGHT " << 1 << "\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS "
            << mapStuff.size()
            << "\n"
            << "DATA ascii\n";
	for (auto p : mapStuff) {
		Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();//ORB_SLAM3::Converter::toVector3d(p->GetWorldPos());
		// std::cout << v.x() << "," << v.y() << "," << v.z() << std::endl;
        ofs << v.x()  << " " << v.y()  << " " << v.z()  << "\n";
	}
    ofs.close();
    cout << "End to write PCD" << endl;
    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC(file_nameTraj);
    SLAM.SaveKeyFrameTrajectoryEuRoC(file_nameKey);
    cout << "Yo Shutting" << endl;
    // Tracking time statistics
    // sort(vTimesTrack.begin(),vTimesTrack.end());
    // float totaltime = 0;
    // for(int ni=0; ni<nImages; ni++)
    // {
    //     totaltime+=vTimesTrack[ni];
    // }
    // cout << "-------" << endl << endl;
    // cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    // cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

// void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                 vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
// {
//     ifstream fAssociation;
//     fAssociation.open(strAssociationFilename.c_str());
//     while(!fAssociation.eof())
//     {
//         string s;
//         getline(fAssociation,s);
//         if(!s.empty())
//         {
//             stringstream ss;
//             ss << s;
//             double t;
//             string sRGB, sD,time;
//             ss >> time;
// 	    time = time.substr(0, t.find_last_of("."));
// 	    time >> t;
//             vTimestamps.push_back(t);
//             ss >> sRGB;
//             vstrImageFilenamesRGB.push_back(sRGB);
//             ss >> t;
//             ss >> sD;
//             vstrImageFilenamesD.push_back(sD);

//         }
//     }
// }
void LoadImagesNew(const string &path_to_data,
                vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimeStamps)
{
    vTimeStamps.reserve(5000);
    vstrImageFilenamesRGB.reserve(5000);
    vstrImageFilenamesD.reserve(5000);
    double tstart = -1;
    std::vector<std::string> files =
            listFilesInDirectoryForCamera(path_to_data + "/rgb", ".png", "color");
    double fileTimeStart = readTimeFromFilename(files[0].substr(
                            files[0].find("_data_") + 6));
    for (int i = 0; i < files.size(); i++){
        double timeStamp = readTimeFromFilename(files[i].substr(
                            files[i].find("_data_") + 6)) -
                        fileTimeStart;
        std::string colorName = path_to_data + "/rgb/"  + files[i] + ".png";
        std::string depthName = path_to_data + "/depth/"  + files[i] + ".png";
        vstrImageFilenamesRGB.push_back(colorName);
        vstrImageFilenamesD.push_back(depthName);
        vTimeStamps.push_back(timeStamp);
    }
}

void LoadImages(const string &strPathRGB, const string &strPathD, const string &strPathTimes,
                vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimeStamps)
{
    std::ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageFilenamesRGB.reserve(5000);
    vstrImageFilenamesD.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageFilenamesRGB.push_back(strPathRGB + "/" + ss.str());
            vstrImageFilenamesD.push_back(strPathD + "/" + ss.str());
            double t;
            string timestring = s.substr(0, s.find_last_of("."));
	    std::string::size_type sz;  
	    // cout << timestring << endl;
        //     timestring >> t;
	    t = std::stod(timestring,&sz);
            vTimeStamps.push_back(t);

        }
    }
}

// Converts UTC time string to a double value.
double readTimeFromFilename(string filename) {
    std::string::size_type sz;
    // Create a stream which we will use to parse the string
    std::istringstream ss(filename);

    // Create a tm object to store the parsed date and time.
    std::tm dt = {0};

    // Now we read from buffer using get_time manipulator
    // and formatting the input appropriately.
    ss >> std::get_time(&dt, "%Y-%m-%dT%H_%M_%SZ");
    double sub_sec =
        (double)std::stof(filename.substr(filename.find(".")), &sz);
    time_t thisTime = std::mktime(&dt);

    double myTime = (double)thisTime + sub_sec;
    return myTime;
}

std::vector<std::string> listFilesInDirectoryForCamera(
    std::string data_directory, std::string extension,
    std::string camera_name) {
    std::vector<std::string> file_paths;
    std::string currFile;
    for (const auto &entry : directory_iterator(data_directory)) {
        currFile = (entry.path()).stem().string();
        if (camera_name == currFile.substr(0, currFile.find("_data_"))) {
            
            file_paths.push_back(currFile);
        }
    }
    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

