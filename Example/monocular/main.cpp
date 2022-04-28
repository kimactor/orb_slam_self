#include <iostream>
#include <stdio.h>
#include <System.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <vector>
#include <chrono>
#include <windows.h>

using namespace std;

void LoadImage(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char** argv) {
    std::cout << "Hello, World!" << std::endl;
    if (argc != 4)
    {
        cerr << endl << " Usage: ./mono_orb path_to_vocabulary path_to_settings path_to_sequence " << endl;
        return 1;
    }

    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3]) + "/rgb.txt";
    LoadImage(strFile, vstrImageFilenames, vTimestamps);
    int nImages = vstrImageFilenames.size();

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << " --------------" << endl;
    cout << " Start processing sequence .." << endl;
    cout << " Images in the sequence: " << nImages << endl << endl;
    int main_error = 0;

    std::thread runThread([&]()
        {
            cv::Mat im;
            for (int ni = 0; ni < nImages; ni++)
            {
                im = cv::imread(string(argv[3]) + "/" + vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
                double tframe = vTimestamps[ni];
                if (im.empty())
                {
                    cerr << endl << "Failed to load image at: " << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
                    main_error = 1;
                    return;
                }
                //cv::imshow("test", im);
                //cv::waitKey(1);
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                SLAM.TrackMonocular(im, tframe);
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

                vTimesTrack[ni] = ttrack;

                double T = 0;
                if (ni < nImages - 1)
                    T = vTimestamps[ni + 1] - tframe;
                else if (ni > 0)
                    T = tframe - vTimestamps[ni - 1];

                if (ttrack < T)
                    Sleep((T - ttrack) * 1e6);

            }
        });

    SLAM.StartViewer();
    cout << "Viewer started, waiting for thread. " << endl;

    runThread.join();
    if (main_error != 0)
        return main_error;
    cout << "Tracking thread joined ..." << endl;

    SLAM.ShutDown();
    cout << " System shutdown" << endl;

    return 0;
}


void LoadImage(const string& strFile, vector<string>& vstrImageFilenames, vector<double>& vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);


    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}