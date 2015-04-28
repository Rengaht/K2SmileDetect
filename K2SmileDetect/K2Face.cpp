
#include "stdafx.h"

#include "K2Face.h"
#include "iostream"


using namespace std;



// define the face frame features required to be computed by this application
static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;


K2Face::K2Face() :
m_pBodyFrameReader(nullptr),
m_pCoordinateMapper(NULL)
{
	
	/*LARGE_INTEGER qpf={0};
	if(QueryPerformanceFrequency(&qpf)){
		cout<<"PerFreq= "<<double(qpf.QuadPart)<<endl;
	}*/

	for(int i=0; i<BODY_COUNT; i++){
		m_pFaceFrameSources[i]=nullptr;
		m_pFaceFrameReaders[i]=nullptr;
	}

	//transmitSocket = new UdpTransmitSocket(IpEndpointName(ADDRESS, PORT));
	/*char testdata[6] = "ABCDE";
	lo_blob btest = lo_blob_new(sizeof(testdata), testdata);*/
	/*lo_address t = lo_address_new("127.0.0.1", "7000");
	lo_send(t, "/jamin/scene", "i", 2);*/
}

K2Face::~K2Face(){
	
	// done with face sources and readers
	for(int i=0; i<BODY_COUNT; i++){
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
	}

	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if(pKinectSensor){
		pKinectSensor->Close();
	}

	SafeRelease(pKinectSensor);


}

void K2Face::Run(){

	HRESULT hr = InitK2Sensor();
	if(SUCCEEDED(hr)){
		while (true){
			Update();
		}
	}

}

void K2Face::Update(){
	
	if(!m_pBodyFrameReader){
		return;
	}
	
	IBodyFrame* pBodyFrame = nullptr;
	HRESULT	hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	IBody* ppBodies[BODY_COUNT] = { 0 };

	if(SUCCEEDED(hr)){
		hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
		if (SUCCEEDED(hr)){
			ProcessFace(BODY_COUNT,ppBodies);
		}
		
		for(int i = 0; i<_countof(ppBodies); ++i){
				SafeRelease(ppBodies[i]);
		}
		
	}
	SafeRelease(pBodyFrame);


}

HRESULT K2Face::InitK2Sensor(){

	cout << "Init Kinect..." << endl;

	HRESULT hr;

	IBodyFrameSource* pBodyFrameSource = nullptr;

	hr=GetDefaultKinectSensor(&pKinectSensor);
	if(FAILED(hr)) return hr;
	
	cout << "Kinect OK!" << endl;

	hr=pKinectSensor->Open();
	if(SUCCEEDED(hr)){
		hr = pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
	}
	if(SUCCEEDED(hr)){
		hr=pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
	}
	if(SUCCEEDED(hr)){
		hr=pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	}
	cout << "Kinect Sensor OK!" << endl;

	if(SUCCEEDED(hr)){
		// create a face frame source + reader to track each body in the fov
		for(int i=0;i<BODY_COUNT;i++){
			if(SUCCEEDED(hr)){
				// create the face frame source by specifying the required face frame features
				hr = CreateFaceFrameSource(pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
			}
			if(SUCCEEDED(hr)){
				// open the corresponding reader
				hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
			}
		}
	}

	SafeRelease(pBodyFrameSource);

	if(!pKinectSensor || FAILED(hr)){
		cout<<"No ready Kinect found!"<<endl;
		return E_FAIL;
	}

	return hr;
}
HRESULT K2Face::UpdateBodyData(IBody** ppBodies){
	HRESULT hr=E_FAIL;

	if(m_pBodyFrameReader!=nullptr){
		IBodyFrame* pBodyFrame=nullptr;
		hr=m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
		if(SUCCEEDED(hr)){
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
		}
		SafeRelease(pBodyFrame);
	}

	return hr;
}

void K2Face::ProcessFace(int nBodyCount, IBody** ppBodies){

	HRESULT hr;
	

	// iterate through each face reader
	for(int iFace=0; iFace<BODY_COUNT; ++iFace){

		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame=nullptr;
		hr=m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked=false;
		if(SUCCEEDED(hr)&& nullptr!=pFaceFrame){
			// check if a valid face is tracked in this face frame
			hr=pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}
		
	

		if(SUCCEEDED(hr)){

			// 1. get head pos
			Joint joints[JointType_Count];
			float screenPointX=0;
			float screenPointY=0;
			IBody* pBody = ppBodies[iFace];
			
			if(pBody){


				BOOLEAN bTracked = false;
				hr = pBody->get_IsTracked(&bTracked);
				
				if (SUCCEEDED(hr) && bTracked){
				
					hr = pBody->GetJoints(_countof(joints), joints);
					if (SUCCEEDED(hr)){
					
						CameraSpacePoint head_point = joints[JointType::JointType_Head].Position;
						float* screen_point = BodyToScreen(head_point);
						screenPointX = screen_point[0];
						screenPointY = screen_point[1];

					}
				}
			}

			// 2. get face 
			if (bFaceTracked){
				IFaceFrameResult* pFaceFrameResult = nullptr;
				RectI faceBox = { 0 };
				PointF facePoints[FacePointType::FacePointType_Count];
				Vector4 faceRotation;
				DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
				//D2D1_POINT_2F faceTextLayout;
				UINT64 tracking_id;
				
				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

				// need to verify if pFaceFrameResult contains data before trying to access it
				if(SUCCEEDED(hr) && pFaceFrameResult != nullptr){
					hr=pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);

					if(SUCCEEDED(hr)){
						hr=pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
					}

					if(SUCCEEDED(hr)){
						hr=pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
					}

					if(SUCCEEDED(hr)){
						hr=pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);
					}
					if (SUCCEEDED(hr)){
						hr = pFaceFrame->get_TrackingId(&tracking_id);
					}
					
					if(SUCCEEDED(hr)){
						//PrintFaceResult(tracking_id,&faceBox,facePoints,faceProperties);
						cout << "Get Face Result: " << tracking_id << endl;
						float score=CalculateSmileScore(tracking_id,&faceBox, facePoints, faceProperties);
						SendFaceResult(tracking_id,screenPointX,screenPointY, &faceBox, score);

					}
				}

				SafeRelease(pFaceFrameResult);

			}else{
				
				
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				//if(bHaveBodyData){
					
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
				
					if(pBody!=nullptr){
						BOOLEAN bTracked=false;
						hr=pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if(SUCCEEDED(hr) && bTracked){
							// get the tracking ID of this body
							hr=pBody->get_TrackingId(&bodyTId);
							if(SUCCEEDED(hr)){
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
								
							}
						}
					}
				//}
			}
		}

		SafeRelease(pFaceFrame);
	}

	
}


void K2Face::PrintFaceResult(const UINT64 tracking_id, const RectI* pFaceBox, const PointF* pFacePoints, const DetectionResult* pFaceProperties){


	boolean ishappy = ((int)pFaceProperties[FaceProperty::FaceProperty_Happy] == 3);
	boolean ismouth_move = ((int)pFaceProperties[FaceProperty::FaceProperty_MouthMoved] == 3);
	boolean ismouth_open = ((int)pFaceProperties[FaceProperty::FaceProperty_MouthOpen] == 3);

	int wid = pFaceBox->Right - pFaceBox->Left;
	int hei = pFaceBox->Bottom - pFaceBox->Top;

	cout <<tracking_id<< endl
		<<pFaceBox->Top << " , " << pFaceBox->Left << " , " << wid << " , " << hei<<endl
		<<"happy: "<<(ishappy?1:0)<<endl
		<<"mouth move: " << (ismouth_move?1:0) << endl
		<<"mouth open: " << (ismouth_open?1:0) << endl;
		

}
void K2Face::SendFaceResult(const UINT64 tracking_id, float head_pos_x,float head_pos_y,const RectI* pFaceBox, float smile_score){
	
	INT32 wid = pFaceBox->Right - pFaceBox->Left;
	INT32 hei = pFaceBox->Bottom - pFaceBox->Top;

	cout << "head_pos =" << head_pos_x << " , " << head_pos_y 
		 << " face_pos = " << pFaceBox->Left << " , " << pFaceBox->Top << endl;
	//if (pFaceBox->Top == 0 && pFaceBox->Left == 0) return;

	lo_address t = lo_address_new(ADDRESS, PORT);
	lo_send(t, "/face", "ifffff", 
			static_cast<int>(tracking_id), smile_score,
			head_pos_x,head_pos_y,
			(float)wid,(float)hei);


}
void K2Face::transmitSmileData(boolean isSmiling){
	
	lo_address t = lo_address_new(ADDRESS,PORT);
	if(isSmiling) lo_send(t, "/smile", "i", 1);
	else lo_send(t, "/smile", "i", 0);
}

float K2Face::CalculateSmileScore(const UINT64 tracking_id, const RectI* pFaceBox, const PointF* pFacePoints, const DetectionResult* pFaceProperties){

	float ishappy = ((int)pFaceProperties[FaceProperty::FaceProperty_Happy] == 3 ? 1.0f : 0.0f);
	float ismouth_move = ((int)pFaceProperties[FaceProperty::FaceProperty_MouthMoved] == 3 ? 1.0f : 0.0f);
	float ismouth_open = ((int)pFaceProperties[FaceProperty::FaceProperty_MouthOpen] == 3 ? 1.0f : 0.0f);
	

	return (ishappy*3.0f+ismouth_move+ismouth_open)/5.0f;

}

float* K2Face::BodyToScreen(const CameraSpacePoint& body_point){

	
	//cout << "head_point= " << body_point.X << ", " << body_point.Y << endl;

	DepthSpacePoint depthPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToDepthSpace(body_point, &depthPoint);
	//cout << "depth_point= " << depthPoint.X << ", " << depthPoint.Y << endl;


	float screenPointX = static_cast<float>(depthPoint.X * 1920) / 512;
	float screenPointY = static_cast<float>(depthPoint.Y * 1080) / 424;
	
	float location[]={screenPointX, screenPointY};
	return location;
}