// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#define _WINSOCKAPI_ 

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

#include <windows.h>


// Kinect Header files
#include <Kinect.h>
#include <Kinect.Face.h>


// TODO: reference additional headers your program requires here


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != nullptr)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}