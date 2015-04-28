#include "stdafx.h"

#include "lo/lo.h"

#define ADDRESS "127.0.0.1"
#define PORT "7000"
#define OUTPUT_BUFFER_SIZE 1024

class K2Face{


	public:
		K2Face();
		~K2Face();

		void Run();

	private:
		void Update();
		HRESULT InitK2Sensor();
		
		void ProcessFace(int nBodyCount, IBody** ppBodies);
		HRESULT UpdateBodyData(IBody** ppBodies);

		void PrintFaceResult(const UINT64 tracking_id, const RectI* pFaceBox, const PointF* pFacePoints, const DetectionResult* pFaceProperties);
		void SendFaceResult(const UINT64 tracking_id, float head_pos_x,float head_pos_y,const RectI* pFaceBox, float smile_score);
		
		float CalculateSmileScore(const UINT64 tracking_id, const RectI* pFaceBox, const PointF* pFacePoints, const DetectionResult* pFaceProperties);

		// Current Kinect
		IKinectSensor* pKinectSensor;

		// Body reader
		IBodyFrameReader*      m_pBodyFrameReader;

		// Face sources
		IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];

		// Face readers
		IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];


		ICoordinateMapper*      m_pCoordinateMapper;

		/*UdpTransmitSocket* transmitSocket;*/
		void transmitSmileData(boolean isSmiling);
		float* BodyToScreen(const CameraSpacePoint& bodyPoint);
};