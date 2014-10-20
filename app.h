#include <iostream>
#include <string>
#include <sstream>
#include "Windows.h" 
#include <opencv2/opencv.hpp>  
#include <Kinect.h> 

using namespace std;
using namespace cv;

class cap
{
	 static const int        cColorWidth  = 1920;
     static const int        cColorHeight = 1080;

	 static const int        cDepthWidth  = 512;
     static const int        cDepthHeight = 424;


public:
	cap(void);

	~cap(void);

	HRESULT getCdata();
	HRESULT getDdata();
	HRESULT getSdata();

	DWORD ConnectKinect();

private:

	    double                  m_fFreq;

	    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;

	//source
	IColorFrameSource* pColorFrameSource;
	IDepthFrameSource* pDepthFrameSource;
	IBodyFrameSource* pBodyFrameSource;

    // reader
    IColorFrameReader*      m_pColorFrameReader;
    IDepthFrameReader*      m_pDepthFrameReader;
    IBodyFrameReader*      m_pBodyFrameReader;

	//description
	IFrameDescription* pDescription;

	unsigned int c_bufferSize;
	unsigned int d_bufferSize;
	Mat c_bufferMat;
	Mat d_bufferMat;
	Mat colorMat;
	Mat depthMat;
};


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}
