#include "app.h"

cap::cap(void):
	pColorFrameSource(NULL),
	pDepthFrameSource(NULL),
	pBodyFrameSource(NULL),
	m_pColorFrameReader(NULL),
	m_pDepthFrameReader(NULL),
	m_pKinectSensor(NULL),
	m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL)
{
	    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

}


cap::~cap(void)
{
	SafeRelease(pColorFrameSource);
    // done with color frame reader
    SafeRelease(m_pColorFrameReader);


	SafeRelease(pDepthFrameSource);
    // done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

	SafeRelease(pBodyFrameSource);
	// done with body frame reader
    SafeRelease(m_pBodyFrameReader);

	SafeRelease( pDescription );

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
	destroyAllWindows();
}


DWORD cap::ConnectKinect()
{
	 HRESULT hr;
    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

		hr = m_pKinectSensor->Open();//打开Kinect
 

	if (NULL == m_pKinectSensor || FAILED(hr))
	{
		cout<<"No ready Kinect found!"<<endl;
		return E_FAIL;
	}
	return hr;
}


HRESULT cap::getDdata()
{
		  	//方法get_DepthFrameSource打开深度帧的源。
			HRESULT hr;
          hr =m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		  	if( FAILED( hr ) ){
			std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
			return -1;
			}

		  	//方法OpenReader打开深度帧读取器。
            hr =pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
			if( FAILED( hr ) ){
				std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
				return -1;
			}


			// Description
			hr = pDepthFrameSource->get_FrameDescription( &pDescription );
			if( FAILED( hr ) ){
				std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
				return -1;
			}
	 
			int d_Width = 0;
			int d_Height = 0;
			pDescription->get_Width( &d_Width ); // 512
			pDescription->get_Height( &d_Height ); // 424
			unsigned int d_bufferSize = d_Width * d_Height * sizeof( unsigned short );

			// Range
			unsigned short min = 0;
			unsigned short max = 0;
			pDepthFrameSource->get_DepthMinReliableDistance( &min ); // 500
			pDepthFrameSource->get_DepthMaxReliableDistance( &max ); // 4500
			cout << "Range : " << min << " - " << max << std::endl;
	
			Mat d_bufferMat( d_Height, d_Width, CV_16UC1 );
			Mat depthMat( d_Height, d_Width, CV_8UC1 );
			namedWindow( "Depth" );

		// Frame
		IDepthFrame* pDepthFrame = nullptr;
		 hr = m_pDepthFrameReader->AcquireLatestFrame( &pDepthFrame );
		if( SUCCEEDED( hr ) ){
			hr = pDepthFrame->AccessUnderlyingBuffer( &d_bufferSize, reinterpret_cast<UINT16**>( &d_bufferMat.data ) );
			if( SUCCEEDED( hr ) ){
				d_bufferMat.convertTo( depthMat, CV_8U, -255.0f / 4500.0f, 255.0f );
			}
		}
		SafeRelease( pDepthFrame );

		imshow( "Depth", depthMat );

		return S_OK;
}


//得到彩色和骨骼点数据
HRESULT cap::getSdata()
{

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open( );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource( &pBodySource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader( &pBodyReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription( &pDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width( &width ); // 1920
	pDescription->get_Height( &height ); // 1080
	unsigned int bufferSize = width * height * 4 * sizeof( unsigned char );

	cv::Mat bufferMat( height, width, CV_8UC4 );
	cv::Mat bodyMat( height / 2, width / 2, CV_8UC4 );
	cv::namedWindow( "Body" );

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255,   0,   0 );
	color[1] = cv::Vec3b(   0, 255,   0 );
	color[2] = cv::Vec3b(   0,   0, 255 );
	color[3] = cv::Vec3b( 255, 255,   0 );
	color[4] = cv::Vec3b( 255,   0, 255 );
	color[5] = cv::Vec3b(   0, 255, 255 );

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
		if( SUCCEEDED( hResult ) ){
			hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize, reinterpret_cast<BYTE*>( bufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
			if( SUCCEEDED( hResult ) ){
				cv::resize( bufferMat, bodyMat, cv::Size(), 0.5, 0.5 );
			}
		}
		//SafeRelease( pColorFrame );

		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
		if( SUCCEEDED( hResult ) ){
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );
			if( SUCCEEDED( hResult ) ){
				for( int count = 0; count < BODY_COUNT; count++ ){
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked( &bTracked );
					if( SUCCEEDED( hResult ) && bTracked ){
						Joint joint[JointType::JointType_Count];
						hResult = pBody[ count ]->GetJoints( JointType::JointType_Count, joint );
						if( SUCCEEDED( hResult ) ){
							// Left Hand State
							HandState leftHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandLeftState( &leftHandState );
							if( SUCCEEDED( hResult ) ){
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace( joint[JointType::JointType_HandLeft].Position, &colorSpacePoint );
								if( SUCCEEDED( hResult ) ){
									int x = static_cast<int>( colorSpacePoint.X );
									int y = static_cast<int>( colorSpacePoint.Y );
									if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
										if( leftHandState == HandState::HandState_Open ){
											cv::circle( bufferMat, cv::Point( x, y ), 75, cv::Scalar( 0, 128, 0 ), 5, CV_AA );
										}
										else if( leftHandState == HandState::HandState_Closed ){
											cv::circle( bufferMat, cv::Point( x, y ), 75, cv::Scalar( 0, 0, 128 ), 5, CV_AA );
										}
										else if( leftHandState == HandState::HandState_Lasso ){
											cv::circle( bufferMat, cv::Point( x, y ), 75, cv::Scalar( 128, 128, 0 ), 5, CV_AA );
										}
									}
								}
							}

							// Right Hand State
							HandState rightHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandRightState( &rightHandState );
							if( SUCCEEDED( hResult ) ){
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace( joint[JointType::JointType_HandRight].Position, &colorSpacePoint );
								if( SUCCEEDED( hResult ) ){
									int x = static_cast<int>( colorSpacePoint.X );
									int y = static_cast<int>( colorSpacePoint.Y );
									if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
										if( rightHandState == HandState::HandState_Open ){
											cv::circle( bufferMat, cv::Point( x, y ), 75, cv::Scalar( 0, 128, 0 ), 5, CV_AA );
										}
										else if( rightHandState == HandState::HandState_Closed ){
											cv::circle( bufferMat, cv::Point( x, y ), 75, cv::Scalar( 0, 0, 128 ), 5, CV_AA );
										}
										else if( rightHandState == HandState::HandState_Lasso ){
											cv::circle( bufferMat, cv::Point( x, y ), 75, cv::Scalar( 128, 128, 0 ), 5, CV_AA );
										}
									}
								}
							}

							// Joint
							for( int type = 0; type < JointType::JointType_Count; type++ ){
								ColorSpacePoint colorSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
								int x = static_cast<int>( colorSpacePoint.X );
								int y = static_cast<int>( colorSpacePoint.Y );
								if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
									cv::circle( bufferMat, cv::Point( x, y ), 5, static_cast< cv::Scalar >( color[count] ), -1, CV_AA );
								}
							}
						}

						/*// Activity
						UINT capacity = 0;
						DetectionResult detectionResults = DetectionResult::DetectionResult_Unknown;
						hResult = pBody[count]->GetActivityDetectionResults( capacity, &detectionResults );
						if( SUCCEEDED( hResult ) ){
							if( detectionResults == DetectionResult::DetectionResult_Yes ){
								switch( capacity ){
									case Activity::Activity_EyeLeftClosed:
										std::cout << "Activity_EyeLeftClosed" << std::endl;
										break;
									case Activity::Activity_EyeRightClosed:
										std::cout << "Activity_EyeRightClosed" << std::endl;
										break;
									case Activity::Activity_MouthOpen:
										std::cout << "Activity_MouthOpen" << std::endl;
										break;
									case Activity::Activity_MouthMoved:
										std::cout << "Activity_MouthMoved" << std::endl;
										break;
									case Activity::Activity_LookingAway:
										std::cout << "Activity_LookingAway" << std::endl;
										break;
									default:
										break;
								}
							}
						}
						else{
							std::cerr << "Error : IBody::GetActivityDetectionResults()" << std::endl;
						}*/

						/*// Appearance
						capacity = 0;
						detectionResults = DetectionResult::DetectionResult_Unknown;
						hResult = pBody[count]->GetAppearanceDetectionResults( capacity, &detectionResults );
						if( SUCCEEDED( hResult ) ){
							if( detectionResults == DetectionResult::DetectionResult_Yes ){
								switch( capacity ){
									case Appearance::Appearance_WearingGlasses:
										std::cout << "Appearance_WearingGlasses" << std::endl;
										break;
									default:
										break;
								}
							}
						}
						else{
							std::cerr << "Error : IBody::GetAppearanceDetectionResults()" << std::endl;
						}*/

						/*// Expression
						capacity = 0;
						detectionResults = DetectionResult::DetectionResult_Unknown;
						hResult = pBody[count]->GetExpressionDetectionResults( capacity, &detectionResults );
						if( SUCCEEDED( hResult ) ){
							if( detectionResults == DetectionResult::DetectionResult_Yes ){
								switch( capacity ){
									case Expression::Expression_Happy:
										std::cout << "Expression_Happy" << std::endl;
										break;
									case Expression::Expression_Neutral:
										std::cout << "Expression_Neutral" << std::endl;
										break;
									default:
										break;
								}
							}
						}
						else{
							std::cerr << "Error : IBody::GetExpressionDetectionResults()" << std::endl;
						}*/

						// Lean
						PointF amount;
						hResult = pBody[count]->get_Lean( &amount );
						if( SUCCEEDED( hResult ) ){
							std::cout << "amount : " << amount.X << ", " << amount.Y << std::endl;
						}
					}
				}
				cv::resize( bufferMat, bodyMat, cv::Size(), 0.5, 0.5 );
			}
			for( int count = 0; count < BODY_COUNT; count++ ){
				SafeRelease( pBody[count] );
			}
		}
		//SafeRelease( pBodyFrame );

		SafeRelease( pColorFrame );
		SafeRelease( pBodyFrame );

		cv::imshow( "Body", bodyMat );


		return S_OK;
}
