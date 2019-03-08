#include "llviewerprecompiledheaders.h"
#include "llviewerVR.h"
#include "llviewerwindow.h"
#include "llwindowwin32.h"
#include "llviewercontrol.h"
#include "llviewercamera.h"
#include "llagentcamera.h"
#include "pipeline.h"
#include "llagent.h"
#include "llkeyboardwin32.h"
#include "llui.h"

#include "llfloaterreg.h"
//#include "llrender.h"

//#include <time.h>
//#include <sys/time.h>
llviewerVR::llviewerVR()
{
	gHMD = NULL;
	gRenderModels = NULL;
	m_kEditKey = KEY_F4;
	m_kZoomKey = KEY_F5;
}


llviewerVR::~llviewerVR()
{
}

/*glh::matrix4f ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t &matPose)
{
glh::matrix4f matrixObj(
matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
);
return matrixObj;
}*/
//unused
vr::HmdQuaternion_t llviewerVR::GetRotation(vr::HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}

LLMatrix4 llviewerVR::ConvertGLHMatrix4ToLLMatrix4(glh::matrix4f m)
{
	LLMatrix4 mout;
	mout.mMatrix[0][0] = m.element(0, 0);
	mout.mMatrix[0][1] = m.element(1, 0);
	mout.mMatrix[0][2] = m.element(2, 0);
	mout.mMatrix[0][3] = m.element(3, 0);

	mout.mMatrix[1][0] = m.element(0, 1);
	mout.mMatrix[1][1] = m.element(1, 1);
	mout.mMatrix[1][2] = m.element(2, 1);
	mout.mMatrix[1][3] = m.element(3, 1);

	mout.mMatrix[2][0] = m.element(0, 2);
	mout.mMatrix[2][1] = m.element(1, 2);
	mout.mMatrix[2][2] = m.element(2, 2);
	mout.mMatrix[2][3] = m.element(3, 2);

	mout.mMatrix[3][0] = m.element(0, 3);
	mout.mMatrix[3][1] = m.element(1, 3);
	mout.mMatrix[3][2] = m.element(2, 3);
	mout.mMatrix[3][3] = m.element(3, 3);
	return mout;
}

glh::matrix4f llviewerVR::ConvertSteamVRMatrixToMatrix42(const vr::HmdMatrix34_t &matPose)
{
	//vr::HmdQuaternion_t q = GetRotation(matPose);

	//gHMDQuat.set(q.x,q.y,q.z,q.w);

	glh::matrix4f matrixObj(
		matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
		matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
		matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
		matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0
		//0, 0, 0, 1.0f
		);

	//LLMatrix4  mat((F32*)matPose.m);
	//gHMDQuat.setQuat(mat);


	//m_nPos.v[0] = matPose.m[0][3];
	//m_nPos.v[1] = matPose.m[2][3];
	//m_nPos.v[2] = matPose.m[1][3];

	//gHMDAxes.mV[0] = atan2(matPose.m[1][0], matPose.m[0][0]);// *57.2957795;//yaw
	//gHMDAxes.mV[2] = atan2(matPose.m[2][1], matPose.m[2][2]);// *57.2957795;//pitch

	//gHMDAxes.mV[0] = matPose.m[2][0];
	//gHMDAxes.mV[1] = matPose.m[2][1];
	//gHMDAxes.mV[2] = matPose.m[2][2];
	return matrixObj;
}

glh::matrix4f llviewerVR::GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye)
{
	if (gHMD == NULL)
		return glh::matrix4f();

	vr::HmdMatrix44_t mat = gHMD->GetProjectionMatrix(nEye, m_fNearClip, m_fFarClip);

	return glh::matrix4f(
		mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
		mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
		mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
		mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
		);
}

glh::matrix4f llviewerVR::GetHMDMatrixPoseEye(vr::Hmd_Eye nEye)
{
	if (gHMD == NULL)
		return glh::matrix4f();

	vr::HmdMatrix34_t matEyeRight = gHMD->GetEyeToHeadTransform(nEye);
	glh::matrix4f matrixObj(
		matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0,
		matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
		matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
		matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f
		);
	glh::matrix4f mt;
	//return matrixObj.inverse();
	//gluInvertMatrix(matrixObj.m, mt.m);
	return mt;
}

//unused
glh::matrix4f llviewerVR::GetCurrentViewProjectionMatrix(vr::Hmd_Eye nEye)
{
	//SetupCameras();
	glh::matrix4f matMVP;
	if (nEye == vr::Eye_Left)
	{
		matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
	}
	else if (nEye == vr::Eye_Right)
	{
		matMVP = m_mat4ProjectionRight * m_mat4eyePosRight *  m_mat4HMDPose;
	}

	return matMVP;
}

//debug func
std::string llviewerVR::MatrixToStr(glh::matrix4f mat, std::string name)
{

	std::string str(name);
	glh::ns_float::vec4 row;
	row = mat.get_row(0);
	str.append("\nLf Row 0 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(-row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));
	str.append(" >\n ");

	row = mat.get_row(1);
	str.append("Up Row 1 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(-row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));
	str.append(" > \n ");

	row = mat.get_row(2);
	str.append("Fw Row 2 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(-row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));
	str.append(" > \n ");

	row = mat.get_row(3);
	str.append("po Row 3 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(-row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));
	str.append(" > \n\n ");






	return str;
}

//Debug func
std::string llviewerVR::MatrixToStrLL(glh::matrix4f mat, std::string name)
{

	std::string str(name);
	glh::ns_float::vec4 row;
	row = mat.get_row(0);
	str.append("\nLf Row 0 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));
	str.append(" >\n ");

	row = mat.get_row(1);
	str.append("Up Row 1 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));

	str.append(" > \n ");

	row = mat.get_row(2);
	str.append("Fw Row 2 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));
	str.append(" > \n ");

	row = mat.get_row(3);
	str.append("po Row 3 =< ");
	str.append(std::to_string(row.v[0]));
	str.append(" , ");
	str.append(std::to_string(row.v[1]));
	str.append(" , ");
	str.append(std::to_string(row.v[2]));
	str.append(" , ");
	str.append(std::to_string(row.v[3]));
	str.append(" > \n\n ");






	return str;
}

bool llviewerVR::gluInvertMatrix(const float m[16], float invOut[16])
{
	float inv[16], det;
	int i;

	inv[0] = m[5] * m[10] * m[15] -
		m[5] * m[11] * m[14] -
		m[9] * m[6] * m[15] +
		m[9] * m[7] * m[14] +
		m[13] * m[6] * m[11] -
		m[13] * m[7] * m[10];

	inv[4] = -m[4] * m[10] * m[15] +
		m[4] * m[11] * m[14] +
		m[8] * m[6] * m[15] -
		m[8] * m[7] * m[14] -
		m[12] * m[6] * m[11] +
		m[12] * m[7] * m[10];

	inv[8] = m[4] * m[9] * m[15] -
		m[4] * m[11] * m[13] -
		m[8] * m[5] * m[15] +
		m[8] * m[7] * m[13] +
		m[12] * m[5] * m[11] -
		m[12] * m[7] * m[9];

	inv[12] = -m[4] * m[9] * m[14] +
		m[4] * m[10] * m[13] +
		m[8] * m[5] * m[14] -
		m[8] * m[6] * m[13] -
		m[12] * m[5] * m[10] +
		m[12] * m[6] * m[9];

	inv[1] = -m[1] * m[10] * m[15] +
		m[1] * m[11] * m[14] +
		m[9] * m[2] * m[15] -
		m[9] * m[3] * m[14] -
		m[13] * m[2] * m[11] +
		m[13] * m[3] * m[10];

	inv[5] = m[0] * m[10] * m[15] -
		m[0] * m[11] * m[14] -
		m[8] * m[2] * m[15] +
		m[8] * m[3] * m[14] +
		m[12] * m[2] * m[11] -
		m[12] * m[3] * m[10];

	inv[9] = -m[0] * m[9] * m[15] +
		m[0] * m[11] * m[13] +
		m[8] * m[1] * m[15] -
		m[8] * m[3] * m[13] -
		m[12] * m[1] * m[11] +
		m[12] * m[3] * m[9];

	inv[13] = m[0] * m[9] * m[14] -
		m[0] * m[10] * m[13] -
		m[8] * m[1] * m[14] +
		m[8] * m[2] * m[13] +
		m[12] * m[1] * m[10] -
		m[12] * m[2] * m[9];

	inv[2] = m[1] * m[6] * m[15] -
		m[1] * m[7] * m[14] -
		m[5] * m[2] * m[15] +
		m[5] * m[3] * m[14] +
		m[13] * m[2] * m[7] -
		m[13] * m[3] * m[6];

	inv[6] = -m[0] * m[6] * m[15] +
		m[0] * m[7] * m[14] +
		m[4] * m[2] * m[15] -
		m[4] * m[3] * m[14] -
		m[12] * m[2] * m[7] +
		m[12] * m[3] * m[6];

	inv[10] = m[0] * m[5] * m[15] -
		m[0] * m[7] * m[13] -
		m[4] * m[1] * m[15] +
		m[4] * m[3] * m[13] +
		m[12] * m[1] * m[7] -
		m[12] * m[3] * m[5];

	inv[14] = -m[0] * m[5] * m[14] +
		m[0] * m[6] * m[13] +
		m[4] * m[1] * m[14] -
		m[4] * m[2] * m[13] -
		m[12] * m[1] * m[6] +
		m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] +
		m[1] * m[7] * m[10] +
		m[5] * m[2] * m[11] -
		m[5] * m[3] * m[10] -
		m[9] * m[2] * m[7] +
		m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
		m[0] * m[7] * m[10] -
		m[4] * m[2] * m[11] +
		m[4] * m[3] * m[10] +
		m[8] * m[2] * m[7] -
		m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
		m[0] * m[7] * m[9] +
		m[4] * m[1] * m[11] -
		m[4] * m[3] * m[9] -
		m[8] * m[1] * m[7] +
		m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
		m[0] * m[6] * m[9] -
		m[4] * m[1] * m[10] +
		m[4] * m[2] * m[9] +
		m[8] * m[1] * m[6] -
		m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (det == 0)
		return false;

	det = 1.0 / det;

	for (i = 0; i < 16; i++)
		invOut[i] = inv[i] * det;

	return true;
}

void llviewerVR::UpdateHMDMatrixPose()
{
	if (gHMD == NULL)
		return;
	/// for somebody asking for the default figure out the time from now to photons.
	/*	float fSecondsSinceLastVsync;
	gHMD->GetTimeSinceLastVsync(&fSecondsSinceLastVsync, NULL);

	float fDisplayFrequency = gHMD->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
	float fFrameDuration = 1.f / fDisplayFrequency;
	float fVsyncToPhotons = gHMD->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);

	float fPredictedSecondsFromNow = fFrameDuration - fSecondsSinceLastVsync + fVsyncToPhotons;*/

	
	
	
	vr::VRCompositor()->WaitGetPoses(gTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);

	m_iValidPoseCount = 0;
	m_strPoseClasses = "";
	for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
	{
		if (gTrackedDevicePose[nDevice].bPoseIsValid)
		{
			m_iValidPoseCount++;
			m_rmat4DevicePose[nDevice] = ConvertSteamVRMatrixToMatrix42(gTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
			if (m_rDevClassChar[nDevice] == 0)
			{
				switch (gHMD->GetTrackedDeviceClass(nDevice))
				{
				case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
				case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
				case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
				case vr::TrackedDeviceClass_GenericTracker:    m_rDevClassChar[nDevice] = 'G'; break;
				case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
				default:                                       m_rDevClassChar[nDevice] = '?'; break;
				}
			}
			m_strPoseClasses += m_rDevClassChar[nDevice];
		}
	}

	if (gTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
	{
		m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
		//gM4HMDPose = ConvertGLHMatrix4ToLLMatrix4(m_mat4HMDPose);
		//gM4HMDPose.invert;
		//gluInvertMatrix(m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd].m, m_mat4HMDPose.m);
		//m_mat4HMDPose.inverse();
	}
}

std::string llviewerVR::GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError )
{
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
	if (unRequiredBufferLen == 0)
		return "";

	char *pchBuffer = new char[unRequiredBufferLen];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
	std::string sResult = pchBuffer;
	delete[] pchBuffer;
	return sResult;
}

void llviewerVR::SetupCameras()
{
	m_mat4ProjectionLeft = GetHMDMatrixProjectionEye(vr::Eye_Left);
	gM4eyeProjectionLeft = ConvertGLHMatrix4ToLLMatrix4(m_mat4ProjectionLeft);

	m_mat4ProjectionRight = GetHMDMatrixProjectionEye(vr::Eye_Right);
	gM4eyeProjectionRight = ConvertGLHMatrix4ToLLMatrix4(m_mat4ProjectionRight);

	m_mat4eyePosLeft = GetHMDMatrixPoseEye(vr::Eye_Left);
	gM4eyePosLeft = ConvertGLHMatrix4ToLLMatrix4(m_mat4eyePosLeft);
	gM4eyePosLeft.invert();

	m_mat4eyePosRight = GetHMDMatrixPoseEye(vr::Eye_Right);
	gM4eyePosRight = ConvertGLHMatrix4ToLLMatrix4(m_mat4eyePosRight);
	gM4eyePosRight.invert();
}

bool llviewerVR::CreateFrameBuffer(int nWidth, int nHeight, FramebufferDesc &framebufferDesc)
{
	glGenFramebuffers(1, &framebufferDesc.m_nRenderFramebufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nRenderFramebufferId);

	glGenRenderbuffers(1, &framebufferDesc.m_nDepthBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, nWidth, nHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);

	glGenTextures(1, &framebufferDesc.m_nRenderTextureId);
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId);
	glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, nWidth, nHeight, true);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId, 0);

	glGenFramebuffers(1, &framebufferDesc.mFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.mFBO);

	glGenTextures(1, &framebufferDesc.m_nResolveTextureId);
	glBindTexture(GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId, 0);

	// check FBO status
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		return false;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}

void llviewerVR::vrStartup(bool is_shutdown)
{
	//static LLCachedControl<bool> vrEn(gSavedSettings, "EnableVR");
	vrEnabled = gPipeline.EnableSteamVR;
	/*hud_textp = (LLHUDText *)LLHUDObject::addHUDObject(LLHUDObject::LL_HUD_TEXT);
	hud_textp->setZCompare(FALSE);
	LLColor4 color(1, 1, 1);
	hud_textp->setColor(color);
	LLVector3 s = LLViewerCamera::getInstance()->getAtAxis();

	hud_textp->setPositionAgent(gAgent.getPositionAgent() - s);
	std::string str("This is the hud test");
	hud_textp->setString(str);
	hud_textp->setHidden(FALSE);*/


	if (vrEnabled)
	{
		if (gHMD == NULL)
		{
			vr::EVRInitError eError = vr::VRInitError_None;
			gHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);

			if (eError != vr::VRInitError_None)
			{
				gHMD = NULL;
				char buf[1024];
				sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
				//return false;
			}
			else
			{
				m_strDriver = "No Driver";
				m_strDisplay = "No Display";

				m_strDriver = GetTrackedDeviceString(gHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
				m_strDisplay = GetTrackedDeviceString(gHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);


			}

			eError = vr::VRInitError_None;
			if (gHMD != NULL)
				gRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);

			if (!gRenderModels)
			{
				gHMD = NULL;
				vr::VR_Shutdown();

				char buf[1024];
				sprintf_s(buf, sizeof(buf), "Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
				//SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
				//return false;
			}
			eError = vr::VRInitError_None;

			if (!vr::VRCompositor())
			{

				char buf[1024];
				sprintf_s(buf, sizeof(buf), "Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
				gHMD = NULL;
				vr::VR_Shutdown();
			}
			if (gHMD != NULL && !gVRInitComplete)
			{
				gVRInitComplete = TRUE;
				vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseSeated);
				gHMD->GetRecommendedRenderTargetSize(&m_nRenderWidth, &m_nRenderHeight);
				m_fNearClip = LLViewerCamera::getInstance()->getNear();
				m_fFarClip = LLViewerCamera::getInstance()->getFar();
				//m_nRenderHeight	=	1440;
				//m_nRenderWidth	=	1440;
				CreateFrameBuffer(m_nRenderWidth, m_nRenderHeight, leftEyeDesc);
				CreateFrameBuffer(m_nRenderWidth, m_nRenderHeight, rightEyeDesc);
				SetupCameras();
				//vr::VRCompositor()->ForceInterleavedReprojectionOn(true);
				//vr::VRCompositor()->SetTrackingSpace(vr::);

				hud_textp = (LLHUDText *)LLHUDObject::addHUDObject(LLHUDObject::LL_HUD_TEXT);
				hud_textp->setZCompare(FALSE);
				LLColor4 color(1, 1, 1);
				hud_textp->setColor(color);
				hud_textp->setHidden(FALSE);
				hud_textp->setMaxLines(-1);
				
				//m_tTimer1.start();
				
				/*if (!LLKeyboard::keyFromString("x", &m_kEditKey))
				{
					// If the call failed, don't match any key.
					//key = KEY_NONE;
				}*/
				
				
				
			}

		}
	}
	else if (gHMD)
	{
		m_bVrActive = FALSE;
		vr::VR_Shutdown();
		gHMD = NULL;
		//m_tTimer1.stop();
		//m_tTimer1.cleanupClass();
	}
}

bool llviewerVR::ProcessVRCamera()
{
	if (gHMD == NULL)
		return FALSE;
	if (m_bVrActive)//gAgentCamera.getCameraMode() == CAMERA_MODE_MOUSELOOK)
	{
		
		if (!m_pCamButtonLeft)
		{
			LLPanel* panelp = NULL;
			panelp=LLPanel::createFactoryPanel("vr_controlls");
			LLRect rc;
			rc.setCenterAndSize(500, 500,200, 200);
			panelp->setRect(rc);
			panelp->setVisible(TRUE);
			panelp->setEnabled(TRUE);
			
			m_pCamera_floater = LLFloaterReg::findTypedInstance<LLFloaterCamera>("camera");
			if (m_pCamera_floater)
			{
				LLStringExplicit lb("keks");
				//LLRect rc;
				//LLButton  *m_pButton1;
				LLButton  *m_pButton;
				for (int i = 0; i < 2; i++)
				{
				
					LLButton::Params p;
					if (i == 0)
					{
						p.name("rot_left");
						p.label("<<");
					}
					else
					{
						p.name("rot_right");
						p.label(">>");
					}


				m_pButton = LLUICtrlFactory::create<LLButton>(p);


				m_pCamera_floater->addChild(m_pButton);
				//panelp->addChild(m_pButton);
				if (i==0)
					rc.setCenterAndSize(20, 20, 30, 20);
				else
					rc.setCenterAndSize(50, 20, 30, 20);
				m_pButton->setRect(rc);
				m_pButton->setVisible(TRUE);
				m_pButton->setEnabled(TRUE);
				if (i==0)
					m_pCamButtonLeft = m_pButton;
				else
					m_pCamButtonRight = m_pButton;
				}
				m_pCamButtonLeft->setCommitCallback(boost::bind(&llviewerVR::buttonCallbackLeft, this));
				m_pCamButtonRight->setCommitCallback(boost::bind(&llviewerVR::buttonCallbackRight, this));

				/*m_pButton1 = m_pCamera_floater->findChild<LLButton>("rot_left");
				//lb.assign("<");
				if (m_pButton1)
				{
				m_pButton1->setLabel(LLStringExplicit("<"));
				m_pButton1->setCommitCallback(boost::bind(&llviewerVR::buttonsCallback, this));
				}*/


				m_pCamStack = m_pCamera_floater->findChild<LLView>("camera_view_layout_stack");
				

				//m_pCamera_floater->getChildList();
				//m_pButton1->
			}
		}
		
		//get window and set its resolution to something that u can render with. 1440 seems to be ok per eye since the glasses rezolution is 1440x1440 per eye.
		//Your monitor need to be  set to a resolution that has vertical height = > then 1140  i set my monitor to 2560x1440. we are doing this since t he max window size cant be biger then screen resolution.
		// Ineed to find a way to bypass that .
		
		if (!leftEyeDesc.IsReady && !rightEyeDesc.IsReady)//Starting rendering with first (left) eye of stereo rendering
		{
			
			int scrsize = GetSystemMetrics(SM_CYSCREEN);
			if (GetSystemMetrics(SM_CXSCREEN) < scrsize)
				scrsize = GetSystemMetrics(SM_CXSCREEN);
			LLWindow * WI;
			WI = gViewerWindow->getWindow();
			WI->getCursorPosition(&m_MousePos);
			LLCoordWindow m_ScrSize;
			WI->getSize(&m_ScrSize);
			if (m_ScrSize.mX != scrsize*0.95 || m_ScrSize.mY != scrsize*0.95)
				m_ScrSize.set(scrsize*0.95, scrsize*0.95);
			WI->setSize(m_ScrSize);
			m_iHalfWidth = m_ScrSize.mX / 2;
			m_iHalfHeight = m_ScrSize.mY / 2;
			m_iThirdWidth = m_ScrSize.mX / 3;
			m_iThirdHeight = m_ScrSize.mY / 3;
			// mt = GetCurrentViewProjectionMatrix(vr::Eye_Left);
			//UpdateHMDMatrixPose();
			gCurrentCameraPos = LLViewerCamera::getInstance()->getOrigin();
			m_vdir_orig = LLViewerCamera::getInstance()->getAtAxis();
			m_vup_orig = LLViewerCamera::getInstance()->getUpAxis();
			m_vleft_orig = LLViewerCamera::getInstance()->getLeftAxis();
			m_vpos = LLViewerCamera::getInstance()->getOrigin();
			
			//convert HMD matrix in to direction vectors that works with SL
			glh::ns_float::vec4 row = m_mat4HMDPose.get_row(2);
			m_vdir.setVec(row.v[0], -row.v[2], row.v[1]);
			row = m_mat4HMDPose.get_row(1);
			m_vup.setVec(row.v[0], -row.v[2], row.v[1]);
			row = m_mat4HMDPose.get_row(0);
			m_vleft.setVec(row.v[0], -row.v[2], row.v[1]);
			row = m_mat4HMDPose.get_row(3);
			gHmdPos.setVec(row.v[0], -row.v[2], row.v[1]);

			//convert HMD euler angles to   to quat rotation
			LLQuaternion q1(m_vdir, m_vleft, m_vup);
			float r1;
			float p1;
			float y1;
			q1.getEulerAngles(&r1, &p1, &y1);

			//get Agent rot in euler angles
			LLQuaternion q = gAgent.getQuat();
			float r2;
			float p2;
			float y2;
			q.getEulerAngles(&r2, &p2, &y2);

			//make a quat of yaw rot of the player
			//change the HMD rotation according to the player facing direction
			LLQuaternion q2;
			q2.setEulerAngles(0, 0, y2 - (m_fCamRotOffset*DEG_TO_RAD));
			q1 = q1*q2;
			gHMDQuat = q1;

			//Experimental steering with headset
			if (y1 > 0 && y2 > 0 && y1 - y2 > (10 * DEG_TO_RAD))
			{
				//gAgent.yaw(y1-(180*DEG_TO_RAD));
				//gAgent.moveYaw(-1,true);
				//gAgentCamera.setYawKey(-1);
				//gAgentCamera.setLookAt(ELookAtType)
			}
			else if (y1 < 0 && y2<0 && fabs(y1) - fabs(y2) >(10 * DEG_TO_RAD))
			{

				//gAgent.yaw(y1-(180 * DEG_TO_RAD));
				//gAgent.moveYaw(1, true);
				//gAgentCamera.setYawKey(1);
			}///////
			//else
			//gAgent.moveYaw(0, false);

			//convert the quats to direction vectors again
			LLMatrix3 m3 = q1.getMatrix3();
			m_vdir = m3.getFwdRow();
			m_vup = m3.getUpRow();
			m_vleft = m3.getLeftRow();
			m_vdir.normalize();
			m_vup.normalize();
			m_vleft.normalize();
			//gHMDAxes.normalize();
			//LLVector3 end;
			
			

			/*m_uiControllerVertcount = 0;
			m_iTrackedControllerCount = 0;

			for (vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice)
			{
				if (!gHMD->IsTrackedDeviceConnected(unTrackedDevice))
					continue;

				if (gHMD->GetTrackedDeviceClass(unTrackedDevice) != vr::TrackedDeviceClass_Controller)
					continue;

				m_iTrackedControllerCount += 1;

				if (!gTrackedDevicePose[unTrackedDevice].bPoseIsValid)
					continue;

				const glh::matrix4f & mat = m_rmat4DevicePose[unTrackedDevice];
				str.append(MatrixToStr(mat, "CTRL MAtrix"));
			}*/
			///LLCoordGL gCtrlscreen[2];



			if (gPipeline.DistortionApplied)
			{
				LLCoordWindow mpos;
				WI->getCursorPosition(&mpos);
				LLCoordGL mcpos = gViewerWindow->getCurrentMouse();
				std::string str;
				/*LLFloaterCamera * m_pCamera_floater = LLFloaterReg::findTypedInstance<LLFloaterCamera>("camera");
				LLView * m_pCh = 0;//LLFloaterReg::findTypedInstance<LLView>("camera_view_layout_stack");
				LLButton * m_pButton=0;
				if (m_pCamera_floater)
				{
					
					m_pCh = m_pCamera_floater->findChild<LLView>("camera_view_layout_stack");
					m_pButton = m_pCamera_floater->findChild<LLButton>("rot_left");

					for (LLView* pChild : *m_pCamera_floater->getChildList())
					{
						if (pChild->getChildCount())// || (pChild->isCtrl()))
						{
							str.append("\n");
							str.append(pChild->getName());
							str.append("  ");
							str.append(std::to_string(pChild->isCtrl()));
							for (LLView* pChild1 : *pChild->getChildList())
							{
								if (pChild1->getChildCount())// || (pChild->isCtrl()))
								{
									str.append(" C= ");
									str.append(pChild1->getName());
									str.append("  ");
									str.append(std::to_string(pChild1->isCtrl()));
									if (pChild1->getName() == "camera_view_layout_stack")
									{
										LLRect rc;
										rc.setCenterAndSize(80, 80, 160,80);
										pChild1->setRect(rc);
									}
									
									for (LLView* pChild2 : *pChild1->getChildList())
									{
										if (pChild2->getChildCount())// || (pChild->isCtrl()))
										{
											str.append(" C2= ");
											str.append(pChild2->getName());
											str.append("  ");
											str.append(std::to_string(pChild2->isCtrl()));
											
										}

									}
								}

							}
						}
						
					}
					//lb.assign("<");
					
					//m_pButton1->
				}*/

				/*str.append(MatrixToStr(m_mat4HMDPose, "HMD MAtrix"));
				str.append(" Cam Pos \n< ");
				str.append(std::to_string(m_vpos.mV[VX]));
				str.append(" , ");
				str.append(std::to_string(m_vpos.mV[VY]));
				str.append(" , ");
				str.append(std::to_string(m_vpos.mV[VZ]));
				str.append(" > ");

				str.append("\nPointe Pos\n< ");
				str.append(std::to_string(gCtrlPos[0].mV[VX]));
				str.append(" , ");
				str.append(std::to_string(gCtrlPos[0].mV[VY]));
				str.append(" , ");
				str.append(std::to_string(gCtrlPos[0].mV[VZ]));
				str.append(" > ");

				str.append("\nPoint Origin\n< ");
				str.append(std::to_string(gCtrlOrigin[0].mV[VX]));
				str.append(" , ");
				str.append(std::to_string(gCtrlOrigin[0].mV[VY]));
				str.append(" , ");
				str.append(std::to_string(gCtrlOrigin[0].mV[VZ]));
				str.append(" > ");*/

				str.append("\n MCoord X=");
				str.append(std::to_string(mpos.mX));
				str.append(" Y=");
				str.append(std::to_string(mpos.mY));

				str.append("\n MCoord X=");
				str.append(std::to_string(mcpos.mX));
				str.append(" Y=");
				str.append(std::to_string(mcpos.mY));

				str.append("\n Coord1 X=");
				str.append(std::to_string(gCtrlscreen[1].mX));
				str.append(" Y=");
				str.append(std::to_string(gCtrlscreen[1].mY));
				str.append("\n Rheight=");
				str.append(std::to_string(m_nRenderHeight));
				str.append("\n Rwidth=");
				str.append(std::to_string(m_nRenderWidth));
				str.append("\n Button=");
				str.append(std::to_string(gButton));

				str.append("\nEye dist\n");
				str.append(std::to_string(gPipeline.EyeDistance));
				str.append("\nFOV \n");
				str.append(std::to_string(LLViewerCamera::getInstance()->getDefaultFOV()));
				

				hud_textp->setString(str);
				LLVector3 end = m_vpos + -m_vdir * 1.0f;
				hud_textp->setPositionAgent(end);
				hud_textp->setDoFade(FALSE);
				hud_textp->setHidden(FALSE);
			}
			else
				hud_textp->setHidden(TRUE);

		}
		/*
		//Set position for hud text rendering 1 meter in front of the camera
		
		//Fill the text 
		

		str.append(std::to_string(r1*RAD_TO_DEG));
		str.append(" , ");
		str.append(std::to_string(p1*RAD_TO_DEG));
		str.append(" , ");
		str.append(std::to_string(y1*RAD_TO_DEG));
		str.append(" , ");
		str.append(std::to_string(y2*RAD_TO_DEG));
		str.append("\n");

		str.append(std::to_string((y1 - y2)*RAD_TO_DEG));
		str.append(" , ");

		//Display the Text
		
		*/
		/*Experimental stuff
		
		//gAgent.yaw(gHMDAxes[0]); that one would point the ll agent in hmd direction??
		//gAgent.movePitch(gHMDAxes[2]);
		//float fov = 110.f * DEG_TO_RAD;
		//LLViewerCamera::getInstance()->setDefaultFOV(fov);
		//LLViewerCamera::getInstance()->updateCameraLocation(pos,up,end);
		
		//LLCamera cam = *LLViewerCamera::getInstance();
		//LLViewerCamera::getInstance()->orthonormalize();
		//LLViewerCamera::updateFrustumPlanes(cam, TRUE);
		//LLViewerCamera::getInstance()->yaw(gHMDAxes.mV[0]);
		//LLViewerCamera::getInstance()->pitch(gHMDAxes.mV[2]);
		
		
		*/
		
		//change the position of the camera placed 64 millimeters apart as your eye lenses settings are.
		//new_fwd_pos is not focusing ritht now it looks stright in front of the eye .. Later it should focus on nearest object using viewer raycasting
		if (m_bEditActive)
		{
			LLViewerCamera::getInstance()->lookDir(m_vdir_orig, m_vup_orig);
			m_vdir = -m_vdir_orig;
			m_vup = m_vup_orig;
			m_vleft = m_vleft_orig;

		}
		else
			LLViewerCamera::getInstance()->lookDir(-m_vdir, m_vup);

		if (gPipeline.EyeDistance > 0)
		{	
			LLVector3 new_fwd_pos = gCurrentCameraPos - m_vdir * 100.0f;
			LLVector3 new_pos;
			if (!leftEyeDesc.IsReady)//change pos for rendering on left eye
			{
				if (m_bEditActive)
					new_pos = m_vpos + m_vleft * (gPipeline.EyeDistance / 2000);
				else
					new_pos = m_vpos - m_vleft * (gPipeline.EyeDistance / 2000);
				LLViewerCamera::getInstance()->updateCameraLocation(new_pos, m_vup, new_fwd_pos);
			}
			else if (!rightEyeDesc.IsReady)////change pos for rendering on right eye
			{
				if (m_bEditActive)
					new_pos = m_vpos - (m_vleft * (gPipeline.EyeDistance / 2000) * 2);
				else
					new_pos = m_vpos + (m_vleft * (gPipeline.EyeDistance / 2000) * 2);
				LLViewerCamera::getInstance()->updateCameraLocation(new_pos, m_vup, new_fwd_pos);


			}
		}
		
		

	}
	return TRUE;
}

void llviewerVR::vrDisplay()
{
	if (gHMD != NULL)
	{
		if (m_bVrActive)//gAgentCamera.getCameraMode() == CAMERA_MODE_MOUSELOOK)
		{
			

			m_iTextureShift = gPipeline.TextureShift;
			
			U32 bx = 0;
			U32 by = 0;
			U32 tx = gPipeline.mScreen.getWidth();
			U32 ty = gPipeline.mScreen.getHeight();

			/*if (gPipeline.EnableSmoothing)
				m_iZoomIndex = 1;
			else
				m_iZoomIndex = 0;*/
			if (m_MousePos.mX > m_iHalfWidth + m_iThirdWidth && m_MousePos.mY < m_iThirdHeight)//up right
			{
				m_iZoomIndex = 4;
			}
			else if (m_MousePos.mX > m_iHalfWidth + m_iThirdWidth && m_MousePos.mY > m_iHalfHeight + m_iThirdHeight)//down right
			{
				m_iZoomIndex = 5;
			}
			else if (m_MousePos.mX <  m_iHalfHeight - m_iThirdWidth && m_MousePos.mY > m_iHalfHeight + m_iThirdHeight)//down left 
			{
				m_iZoomIndex = 6;
			}
			else if (m_MousePos.mX <  m_iHalfHeight - m_iThirdWidth && m_MousePos.mY < m_iThirdHeight)//up left
			{
				m_iZoomIndex = 7;
			}
			else if (m_MousePos.mX > m_iHalfWidth / 2 && m_MousePos.mX < m_iHalfWidth + (m_iHalfWidth / 2) && m_MousePos.mY > (m_iHalfHeight / 2) && m_MousePos.mY < m_iHalfHeight + (m_iHalfHeight / 2))//up left
			{
				m_iZoomIndex = 0;
			}
			S32 offset = tx / 3;
			if (m_iZoomIndex == 1)
			{
				bx -= tx/4;
				by -= ty/4;
				tx += tx/4;
				ty += ty/4;
			}
			else if (m_iZoomIndex == 2)
			{
				bx -= tx / 3;
				by -= ty / 3;
				tx += tx / 3;
				ty += ty / 3;
			}
			else if (m_iZoomIndex == 3)
			{
				bx -= tx / 2;
				by -= ty / 2;
				tx += tx / 2;
				ty += ty / 2;
			}
			else if (m_iZoomIndex == 4)//up right
			{
				bx = (tx / 3) - offset;
				by = (ty / 3) - offset;
				tx = tx + offset;
				ty = ty + offset;
			}
			else if (m_iZoomIndex == 5)//down right
			{
				bx = (tx / 3) - offset;
				by -= offset;
				tx = tx + offset;
				ty = (ty / 2) + offset;
			}
			else if (m_iZoomIndex == 6)//down left 
			{
				bx -= offset;
				by -= offset;
				tx = (tx / 2) + offset;
				ty = (ty / 2) + offset;
			}
			else if (m_iZoomIndex == 7)//up left
			{
				bx -= offset;
				by = (ty / 2) - offset;
				tx = (tx / 2) + offset;
				ty = ty + offset;
			}

			glDisable(GL_MULTISAMPLE);
			//set the buffer we are reading from . SL rendered to this until now.
			//glReadBuffer(gPipeline.mScreen.sCurFBO);
			glReadBuffer(GL_BACK);
			//if left camera was active bind left eye buffer for drawing in to
			if (!leftEyeDesc.IsReady)
			{
				glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.mFBO);
				if (m_iZoomIndex)
					glClear(GL_COLOR_BUFFER_BIT);
				//leftEyeDesc.IsReady = TRUE;
				
				glBlitFramebuffer(bx, by, tx, ty, m_iTextureShift, 0, m_nRenderWidth + m_iTextureShift, m_nRenderHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR);
				
			}
			if ((leftEyeDesc.IsReady && !rightEyeDesc.IsReady) || gPipeline.EyeDistance == 0)//if right camera was active bind left eye buffer for drawing in to
			{
				glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.mFBO);
				if (m_iZoomIndex)
					glClear(GL_COLOR_BUFFER_BIT);
				rightEyeDesc.IsReady = TRUE;
				glBlitFramebuffer(bx, by, tx, ty, -m_iTextureShift, 0, m_nRenderWidth - m_iTextureShift, m_nRenderHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR);
			}
			if (!leftEyeDesc.IsReady)
				leftEyeDesc.IsReady = TRUE;

			//Remove bindings of read and draw buffer
			glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

			//glBindFramebuffer(0,gPipeline.mScreen.sCurFBO);

			glEnable(GL_MULTISAMPLE);
			//glClear(GL_COLOR_BUFFER_BIT);
			
			
			if (leftEyeDesc.IsReady && (rightEyeDesc.IsReady || gPipeline.EyeDistance == 0))
			{

				rightEyeDesc.IsReady = FALSE;
				leftEyeDesc.IsReady = FALSE;
				//glFlush();
				
				//vr::VRCompositor()->CompositorBringToFront();   could help with no image issues
				

				//Update HMD . !!!!!  This calls waitGetPoses() which is essential to start the rendering process in the HMD after Submit and gets the current HMD pose(rotation location matrix)
				//if you do not call that anywhere no image will be processed. 
				
				
				//submit the textures to the HMD
				//vr::Texture_t lEyeTexture;
				//vr::Texture_t rEyeTexture;
				lEyeTexture = { (void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
				eError = vr::VRCompositor()->Submit(vr::Eye_Left, &lEyeTexture, 0, (vr::EVRSubmitFlags)(vr::Submit_Default ));
				rEyeTexture = { (void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
				eError = vr::VRCompositor()->Submit(vr::Eye_Right, &rEyeTexture, 0, (vr::EVRSubmitFlags)(vr::Submit_Default));
				vr::VRCompositor()->PostPresentHandoff();// Here we tell the HMD  that rendering is done and it can render the image in to the HMD
				glFlush();
				glFinish();
				
				gViewerWindow->getWindow()->swapBuffers();
				
				
				//glFlush();
				
				
				
				UpdateHMDMatrixPose();
				//

			}

		}





	}
	//else if (vrEnabled)
	//{
		//vrStartup();
	//}

}

void llviewerVR::ProcessVREvent(const vr::VREvent_t & event)//process vr´events 
{
	switch (event.eventType)
	{
	case vr::VREvent_TrackedDeviceActivated:
	{
		//SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
		//dprintf("Device %u attached. Setting up render model.\n", event.trackedDeviceIndex);
	}
	break;
	case vr::VREvent_TrackedDeviceDeactivated:
	{
		//dprintf("Device %u detached.\n", event.trackedDeviceIndex);
	}
	break;
	case vr::VREvent_TrackedDeviceUpdated:
	{
		//dprintf("Device %u updated.\n", event.trackedDeviceIndex);
	}
	break;
	}
}

void llviewerVR::agentYaw(F32 yaw_inc)  // move avatar forward backward and rotate 
{
	// Cannot steer some vehicles in mouselook if the script grabs the controls
	if (gAgentCamera.cameraMouselook()  && gSavedSettings.getBOOL("JoystickMouselookYaw"))
	{
		gAgent.rotate(-yaw_inc, gAgent.getReferenceUpVector());
		
	}
	else
	{
		if (yaw_inc < 0)
		{
			gAgent.setControlFlags(AGENT_CONTROL_YAW_POS);
		}
		else if (yaw_inc > 0)
		{
			gAgent.setControlFlags(AGENT_CONTROL_YAW_NEG);
		}

		gAgent.yaw(-yaw_inc);
	}
}

bool llviewerVR::HandleInput()// handles controller input for now  only the stick.
{

	if (gHMD == NULL || !m_bVrActive)
		return FALSE;
	bool bRet = false;

	// Process SteamVR events
	vr::VREvent_t event;
	while (gHMD->PollNextEvent(&event, sizeof(event)))
	{
		ProcessVREvent(event);
	}

	// Process SteamVR controller state
	/*for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
	{
		vr::VRControllerState_t state;
		if (gHMD->GetControllerState(unDevice, &state, sizeof(state)))
		{
			m_rbShowTrackedDevice[unDevice] = state.ulButtonPressed == 0;
			if (state.unPacketNum != gPacketNum)
			{
				gPacketNum = state.unPacketNum;
				//add intensity slider here.
				if (fabs(state.rAxis[2].x) > 0.3)// +x rechts +y fwd
					agentYaw(state.rAxis[2].x / 20);
				if (state.rAxis[2].y > 0.5)// +x rechts +y fwd
					gAgent.moveAt(1, false);
				else if (state.rAxis[2].y < -0.5)// +x rechts +y fwd
					gAgent.moveAt(-1, false);
				gButton = state.ulButtonPressed;
				
				
				
				LLWindow * WI;
				WI = gViewerWindow->getWindow();
				MASK mask = gKeyboard->currentMask(TRUE);		
				//S32 width = gViewerWindow->getWorldViewWidthScaled();
				//S32 height = gViewerWindow->getWindowHeightScaled();
				S32 height = gViewerWindow->getWorldViewHeightScaled();
				LLCoordWindow size;
				size.mX = gCtrlscreen[unDevice].mX;
				//size.mY = gCtrlscreen[unDevice].mY;
				size.mY = height - gCtrlscreen[unDevice].mY;
				//gCtrlscreen[unDevice].mY = height - gCtrlscreen[unDevice].mY;
				

				if ((state.ulButtonPressed &  vr::ButtonMaskFromId(vr::k_EButton_Grip)) && !gRightClick[unDevice])
				{
					
					gRightClick[unDevice] = TRUE;
					if (gAgentCamera.getCameraMode() != CAMERA_MODE_MOUSELOOK)
					{ 
						
						WI->setCursorPosition(size);
					}
					
					//LLWindowWin32 *window_imp = (LLWindowWin32 *)GetWindowLongPtr(mAppWindowHandle, GWLP_USERDATA);
					
					gViewerWindow->handleAnyMouseClick(WI, gCtrlscreen[unDevice], mask, LLMouseHandler::CLICK_RIGHT, TRUE);
					
					
					INPUT Inputs[1] { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
					//SendInput(1, Inputs, sizeof(INPUT));
				}
				else if (gRightClick[unDevice] && !(state.ulButtonPressed &  vr::ButtonMaskFromId(vr::k_EButton_Grip)))
				{
					gRightClick[unDevice] = FALSE;

					gViewerWindow->handleAnyMouseClick(WI, gCtrlscreen[unDevice], mask, LLMouseHandler::CLICK_RIGHT, FALSE);
					INPUT Inputs[1] = { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_RIGHTUP;
					//SendInput(1, Inputs, sizeof(INPUT));


				}
				

				if ((state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) && !gLeftClick[unDevice])
				{
					if (gAgentCamera.getCameraMode() != CAMERA_MODE_MOUSELOOK)
					{
						gLeftClick[unDevice] = TRUE;
						//LLWindow * WI;
						//WI = gViewerWindow->getWindow();
						//S32 width = gViewerWindow->getWorldViewWidthScaled();
						//S32 height = gViewerWindow->getWorldViewHeightScaled();
						//LLCoordWindow size;
						//size.mX = gCtrlscreen[0].mX;
						//size.mY = height - gCtrlscreen[0].mY;
						WI->setCursorPosition(size);
					}
					INPUT Inputs[1] = { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
					SendInput(1, Inputs, sizeof(INPUT));
				}
				else if (gLeftClick[unDevice] && !(state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)))
				{
					gLeftClick[unDevice] = FALSE;
					INPUT Inputs[1] = { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_LEFTUP;
					SendInput(1, Inputs, sizeof(INPUT));
				}

			}

		}
		
	}*/

	return bRet;
}

void llviewerVR::DrawCursors()
{
	gUIProgram.bind();
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//gViewerWindow->setup2DRender();
	gGL.pushMatrix();
	S32 half_width = (gViewerWindow->getWorldViewWidthScaled() / 2);
	S32 half_height = (gViewerWindow->getWorldViewHeightScaled() / 2);

	S32 wwidth = gViewerWindow->getWindowWidthScaled();
	S32 wheight = gViewerWindow->getWindowHeightScaled();

	//translatef moves 0 vector to the pos you specified so oyu can draw fron zero vector there
	gGL.translatef((F32)half_width, (F32)half_height, 0.f);
	gGL.color4fv(LLColor4::white.mV);
	//glClear(GL_DEPTH_BUFFER_BIT);
	//glDisable(GL_DEPTH_TEST);
	LLWindow * WI;
	WI = gViewerWindow->getWindow();
	LLCoordWindow   mcpos;
	WI->getCursorPosition(&mcpos);
	LLCoordGL mpos = gViewerWindow->getCurrentMouse();

	for (vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice)
	{
		if (gCtrlscreen[unTrackedDevice].mX > -1)
		{

			gl_circle_2d(gCtrlscreen[unTrackedDevice].mX - half_width, gCtrlscreen[unTrackedDevice].mY - half_height + gCursorDiff, half_width / 200, 8, TRUE);
		}

	}

	if (gAgentCamera.getCameraMode() != CAMERA_MODE_MOUSELOOK)
	{
		LLColor4 cl;
		cl = LLColor4::white.mV;

		S32 mx = mpos.mX - half_width;
		S32 my = mpos.mY - (half_height);
		if (mpos.mX < 0 || mpos.mX > wwidth)
			mx = half_width;
		if (mpos.mY < 0 || mpos.mY > wheight)
			my = half_height;
		gl_triangle_2d(mx, my, mx + 8, my - 15, mx + 15, my - 8, cl, TRUE);
	}

	//gl_circle_2d(mpos.mX - half_width, mpos.mY - (half_height)  /*+ gVR.gCursorDiff)*/, half_width / 200, 8, TRUE);

	//glEnable(GL_DEPTH_TEST);
	gGL.popMatrix();
	gUIProgram.unbind();
	stop_glerror();
}

void llviewerVR::RenderControllerAxes()
{
	// Don't attempt to update controllers if input is not available
	//gCtrlNum = 0;
	
	if (gHMD == NULL)
		return;
	if (gKeyboard->getKeyDown(KEY_TAB) && !m_bVrKeyDown)
	{
		
		m_bVrKeyDown = TRUE;
		
	}
	else if (!gKeyboard->getKeyDown(KEY_TAB) && m_bVrKeyDown)
	{
		m_bVrKeyDown = FALSE;
		if (gKeyboard->getKeyDown(KEY_CONTROL))
		{
			m_iZoomIndex++;
			if (m_iZoomIndex > 7)
				m_iZoomIndex = 0;
		}
		else
		{
			if (!m_bVrActive)
				m_bVrActive = TRUE;
			else
				m_bVrActive = FALSE;
			LLViewerCamera::getInstance()->setDefaultFOV(1.8);
		}
	}
	if (gKeyboard->getKeyDown(m_kEditKey) && !m_bEditKeyDown)
	{
		m_bEditKeyDown = TRUE;
		//m_iClockCount2 =  m_tTimer1.getCurrentClockCount();
	}
	else if (!gKeyboard->getKeyDown(m_kEditKey) && m_bEditKeyDown)
	{
		m_bEditKeyDown = FALSE;
		//m_iClockCount = m_tTimer1.getCurrentClockCount() - m_iClockCount2;
		/*if (m_iClockCount  > 5000000)
		{
			m_iZoomIndex++;
			if (m_iZoomIndex > 5)
				m_iZoomIndex = 0;
		}
		else*/
			if (!m_bEditActive)
				m_bEditActive = TRUE;
			else
				m_bEditActive = FALSE;
			
			
				

	}
	if (gKeyboard->getKeyDown(m_kZoomKey) && !m_bZoomKeyDown)
	{
		m_bZoomKeyDown = TRUE;
	}
	else if (!gKeyboard->getKeyDown(m_kZoomKey) && m_bZoomKeyDown)
	{
		m_bZoomKeyDown = FALSE;
		m_iZoomIndex++;
		if (m_iZoomIndex > 7)
			m_iZoomIndex = 0;
	}
	if (!gHMD->IsInputAvailable() || !m_bVrActive)
		return;

	//std::vector<float> vertdataarray;



	//m_uiControllerVertcount = 0;
	//m_iTrackedControllerCount = 0;
	
	
	for (vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice)
	{
		gCtrlscreen[unTrackedDevice].set(-1, -1);
		if (!gHMD->IsTrackedDeviceConnected(unTrackedDevice))
			continue;

		if (gHMD->GetTrackedDeviceClass(unTrackedDevice) != vr::TrackedDeviceClass_Controller)
			continue;

		m_iTrackedControllerCount += 1;

		if (!gTrackedDevicePose[unTrackedDevice].bPoseIsValid)
			continue;

		//Count the controllers
		
		const glh::matrix4f & mat = m_rmat4DevicePose[unTrackedDevice];

		//glh::vec4f center; 
		//mat.mult_matrix_vec(glh::vec4f(0, 0, 0, 1),center) ;
		
		LLVector3 pos = LLViewerCamera::getInstance()->getOrigin();
		LLVector3 dir;
		LLVector3 up;
		LLVector3 left;
		
		glh::ns_float::vec4 row = mat.get_row(2);
		dir.setVec(row.v[0], -row.v[2], row.v[1]);

		row = mat.get_row(1);
		up.setVec(row.v[0], -row.v[2], row.v[1]);

		row = mat.get_row(0);
		left.setVec(row.v[0], -row.v[2], row.v[1]);

		row = mat.get_row(3);
		gCtrlOrigin[unTrackedDevice].setVec(row.v[0], -row.v[2], row.v[1]);

		LLQuaternion q1(dir, left, up);
		
		//get Agent rot in euler angles
		float r2;
		float p2;
		float y2;
		gAgent.getQuat().getEulerAngles(&r2, &p2, &y2);



		//make a quat of yaw rot of the player
		LLQuaternion q2;
		q2.setEulerAngles(0, 0, y2 - (90 * DEG_TO_RAD));
	
		//change the controller rotation according to the player facing direction
		LLQuaternion q3;
		q3 = q1*q2;

		//grab the forward vector from the quat matrix
		LLMatrix3 m3 = q3.getMatrix3();
		dir = m3.getFwdRow();

		//up = m3.getUpRow();
		//left = m3.getLeftRow();
		dir.normalize();
		//up.normalize();
		//left.normalize();
		//get position of the controller
		gCtrlOrigin[unTrackedDevice] -= gHmdPos;
		gCtrlOrigin[unTrackedDevice] = gCtrlOrigin[unTrackedDevice] * q2;
		//project 10 meter line  in the direction the controller is facing
		gCtrlPos[unTrackedDevice] = gCtrlOrigin[unTrackedDevice] + -dir * 10.0f;

		//translate the fwd vector line to screen coords
		posToScreen(gCurrentCameraPos + gCtrlPos[unTrackedDevice], gCtrlscreen[unTrackedDevice], FALSE);
	
		//adjust the pos so it fits with the actual mouse cursor pos
		S32 height = gViewerWindow->getWorldViewHeightScaled();
		gCursorDiff= gViewerWindow->getWindowHeightScaled();
		gCursorDiff = gCursorDiff - height;
		gCtrlscreen[unTrackedDevice].mY -= gCursorDiff;
		
	


	
		//draw the controller lines in world  ( make tham nicer ;>)
		LLGLSUIDefault gls_ui;
		gGL.getTexUnit(0)->unbind(LLTexUnit::TT_TEXTURE);
		LLVector3 v = gCurrentCameraPos;	
		// Some coordinate axes
		glClear(GL_DEPTH_BUFFER_BIT);
		glDisable(GL_DEPTH_TEST);
		gGL.pushMatrix();
		gGL.translatef(v.mV[VX], v.mV[VY], v.mV[VZ]);
		gGL.begin(LLRender::LINES);
		gGL.color3f(1.0f, 0.0f, 0.0f);   // i direction = X-Axis = red
		gGL.vertex3f(gCtrlOrigin[unTrackedDevice].mV[VX], gCtrlOrigin[unTrackedDevice].mV[VY], gCtrlOrigin[unTrackedDevice].mV[VZ]);
		gGL.vertex3f(gCtrlPos[unTrackedDevice].mV[VX], gCtrlPos[unTrackedDevice].mV[VY], gCtrlPos[unTrackedDevice].mV[VZ]);
		gGL.end();
		gGL.popMatrix();
		glEnable(GL_DEPTH_TEST);

		
		//read the input from the available controllers
		vr::VRControllerState_t state;
		if (gHMD->GetControllerState(unTrackedDevice, &state, sizeof(state)))
		{
			m_rbShowTrackedDevice[unTrackedDevice] = state.ulButtonPressed == 0;
			if (1)// state.unPacketNum != gPacketNum)
			{
				//if(LLFloaterCamera::inFreeCameraMode())
				gPacketNum = state.unPacketNum;
				//Get the joystick hat state of the controller and move the avatar.. (Figure out how to map it tpo vive and oculus)
				//add movement intensity slider here.
				if (fabs(state.rAxis[2].x) > 0.5)// +x rechts +y fwd
				{
					if (LLFloaterCamera::inFreeCameraMode())
					{
						m_fCamRotOffset += 0.5;
						if (m_fCamRotOffset > 360)
							m_fCamRotOffset = 0;

					}
					else
					{
						m_fCamRotOffset = 90;
						agentYaw(state.rAxis[2].x / 40);

					}
							
				}
				else if (state.rAxis[2].y > 0.5)// +y forward
				{
					if (LLFloaterCamera::inFreeCameraMode())
					{
						
						m_fCamPosOffset += 0.2;
					}
					else
					{
						gAgent.moveAt(1, false);
						m_fCamPosOffset = 0;
					}
						
				}
					
				else if (state.rAxis[2].y < -0.5)// -y back
				{
					if (LLFloaterCamera::inFreeCameraMode())
					{
						
						m_fCamPosOffset -= 0.2;
					}
					else
					{
						gAgent.moveAt(-1, false);
						m_fCamPosOffset = 0;
					}
						
				}
					
				
				
				gButton = state.ulButtonPressed;
				LLWindow * WI;
				WI = gViewerWindow->getWindow();
				//MASK mask = gKeyboard->currentMask(TRUE);
				

				S32 width = gViewerWindow->getWorldViewWidthScaled();
				//S32 height = gViewerWindow->getWindowHeightScaled();
				S32 height = gViewerWindow->getWorldViewHeightScaled();
				LLCoordWindow cpos;
				cpos.mX = gCtrlscreen[unTrackedDevice].mX;
				//size.mY = gCtrlscreen[unDevice].mY;
				cpos.mY = height - gCtrlscreen[unTrackedDevice].mY;
				//gCtrlscreen[unDevice].mY = height - gCtrlscreen[unDevice].mY;

				//LLCoordWindow *  mcpos;
				//WI->getCursorPosition(mcpos);

				//Emulate mouse clicks with the controllers trigger and grip buttons

				if ((state.ulButtonPressed &  vr::ButtonMaskFromId(vr::k_EButton_Grip)) && !gRightClick[unTrackedDevice])
				{

					gRightClick[unTrackedDevice] = TRUE;
					if (gAgentCamera.getCameraMode() != CAMERA_MODE_MOUSELOOK)
					{

						WI->setCursorPosition(cpos);
					}

					//LLWindowWin32 *window_imp = (LLWindowWin32 *)GetWindowLongPtr(mAppWindowHandle, GWLP_USERDATA);

					//gViewerWindow->handleAnyMouseClick(WI, gCtrlscreen[unTrackedDevice], mask, LLMouseHandler::CLICK_RIGHT, TRUE);


					INPUT Inputs[1] { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
					SendInput(1, Inputs, sizeof(INPUT));
				}
				else if (gRightClick[unTrackedDevice] && !(state.ulButtonPressed &  vr::ButtonMaskFromId(vr::k_EButton_Grip)))
				{
					gRightClick[unTrackedDevice] = FALSE;

					//gViewerWindow->handleAnyMouseClick(WI, gCtrlscreen[unTrackedDevice], mask, LLMouseHandler::CLICK_RIGHT, FALSE);
					INPUT Inputs[1] = { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_RIGHTUP;
					SendInput(1, Inputs, sizeof(INPUT));


				}


				if ((state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) && !gLeftClick[unTrackedDevice])
				{
					gLeftClick[unTrackedDevice] = TRUE;
					if (gAgentCamera.getCameraMode() != CAMERA_MODE_MOUSELOOK)
					{
						
						//LLWindow * WI;
						//WI = gViewerWindow->getWindow();
						//S32 width = gViewerWindow->getWorldViewWidthScaled();
						//S32 height = gViewerWindow->getWorldViewHeightScaled();
						//LLCoordWindow size;
						//size.mX = gCtrlscreen[0].mX;
						//size.mY = height - gCtrlscreen[0].mY;
						WI->setCursorPosition(cpos);
					}
					INPUT Inputs[1] = { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
					SendInput(1, Inputs, sizeof(INPUT));
				}
				else if (gLeftClick[unTrackedDevice] && !(state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)))
				{
					gLeftClick[unTrackedDevice] = FALSE;
					INPUT Inputs[1] = { 0 };
					Inputs[0].type = INPUT_MOUSE;
					Inputs[0].mi.dwFlags = MOUSEEVENTF_LEFTUP;
					SendInput(1, Inputs, sizeof(INPUT));
				}

				if(gAgentCamera.getCameraMode() != CAMERA_MODE_MOUSELOOK && gLeftClick[unTrackedDevice] && cpos.mX>-1 && cpos.mX < width  && cpos.mY >-1 && cpos.mY < height)
				{
					
					WI->setCursorPosition(cpos);
				}

			}

		}
	}
	
}

BOOL llviewerVR::posToScreen(const LLVector3 &pos_agent, LLCoordGL &out_point, const BOOL clamp) const
{
	//BOOL in_front = TRUE;
	GLdouble	x, y, z;			// object's window coords, GL-style

	/*LLVector3 dir_to_point = pos_agent - LLViewerCamera::getInstance()->getOrigin();
	dir_to_point /= dir_to_point.magVec();

	if (dir_to_point * LLCoordFrame::getAtAxis() < 0.f)
	{
		if (clamp)
		{
			return FALSE;
		}
		else
		{
			in_front = FALSE;
		}
	}
	*/
	LLRect world_view_rect = gViewerWindow->getWorldViewRectRaw();
	
	//LLRect world_view_rect = gViewerWindow->handleAnyMouseClick;
	
	S32	viewport[4];
	viewport[0] = world_view_rect.mLeft;
	viewport[1] = world_view_rect.mBottom;
	viewport[2] = world_view_rect.getWidth();
	viewport[3] = world_view_rect.getHeight();

	F64 mdlv[16];
	F64 proj[16];

	for (U32 i = 0; i < 16; i++)
	{
		mdlv[i] = (F64)gGLModelView[i];
		proj[i] = (F64)gGLProjection[i];
	}

	if (GL_TRUE == gluProject(pos_agent.mV[VX], pos_agent.mV[VY], pos_agent.mV[VZ],
		mdlv, proj, (GLint*)viewport,
		&x, &y, &z))
	{
		// convert screen coordinates to virtual UI coordinates
		x /= gViewerWindow->getDisplayScale().mV[VX];
		y /= gViewerWindow->getDisplayScale().mV[VY];

		// should now have the x,y coords of grab_point in screen space
		LLRect world_rect = gViewerWindow->getWorldViewRectScaled();

		// convert to pixel coordinates
		S32 int_x = lltrunc(x);
		S32 int_y = lltrunc(y);

		out_point.mX = int_x;
		out_point.mY = int_y;

		BOOL valid = TRUE;
		return valid;
		/*
		if (clamp)
		{
			if (int_x < world_rect.mLeft)
			{
				out_point.mX = world_rect.mLeft;
				valid = FALSE;
			}
			else if (int_x > world_rect.mRight)
			{
				out_point.mX = world_rect.mRight;
				valid = FALSE;
			}
			else
			{
				out_point.mX = int_x;
			}

			if (int_y < world_rect.mBottom)
			{
				out_point.mY = world_rect.mBottom;
				valid = FALSE;
			}
			else if (int_y > world_rect.mTop)
			{
				out_point.mY = world_rect.mTop;
				valid = FALSE;
			}
			else
			{
				out_point.mY = int_y;
			}
			return valid;
		}
		else
		{
			out_point.mX = int_x;
			out_point.mY = int_y;

			if (int_x < world_rect.mLeft)
			{
				valid = FALSE;
			}
			else if (int_x > world_rect.mRight)
			{
				valid = FALSE;
			}
			if (int_y < world_rect.mBottom)
			{
				valid = FALSE;
			}
			else if (int_y > world_rect.mTop)
			{
				valid = FALSE;
			}

			return in_front && valid;
		}*/
	}
	else
	{
		return FALSE;
	}
}

void  llviewerVR::buttonCallbackLeft()
{
	if (m_pCamStack)
	{
		
		m_fCamRotOffset -= 5;
		if (m_fCamRotOffset > 360)
			m_fCamRotOffset = 0;

	}
}

void  llviewerVR::buttonCallbackRight()
{
	if (m_pCamStack)
	{
		m_fCamRotOffset += 5;
		if (m_fCamRotOffset > 360)
			m_fCamRotOffset = 0;
		
		LLRect rc;
		rc.setCenterAndSize(80, 80, 160, 80);
		m_pCamStack->setRect(rc);
		if (m_pCamera_floater)
		{
			rc = m_pCamera_floater->getRect();
			rc.setCenterAndSize(rc.getCenterX(), rc.getCenterY(), 200, 120);
			//m_pCamera_floater->setRect(rc);
			m_pCamera_floater->handleReshape(rc, TRUE);
		}
		

	}
}

