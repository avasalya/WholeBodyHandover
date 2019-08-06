/*=========================================================
 //
 // File: ClientTest.cpp
 //
 // Created by Ned Phipps, Oct-2004
 //
 =============================================================================*/

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <vector>

#include "cortex.h"



using namespace std;

void MyErrorMsgHandler(int iLevel, const char *szMsg)
{
  const char *szLevel = NULL;

  if (iLevel == VL_Debug) {
    szLevel = "Debug";
  } else if (iLevel == VL_Info) {
    szLevel = "Info";
  } else if (iLevel == VL_Warning) {
    szLevel = "Warning";
  } else if (iLevel == VL_Error) {
    szLevel = "Error";
  }
  printf("  %s: %s\n", szLevel, szMsg);
}


int main()
{

  sBodyDefs* pBodyDefs = NULL;
  sFrameOfData* pFrameOfData = NULL;
  
  std::vector<int> bodyMarkers;

  int retval = RC_Okay;

  Cortex_SetVerbosityLevel(VL_Info);
  Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
  
  retval = Cortex_Initialize("10.1.1.200", "10.1.1.190");
  if (retval != RC_Okay)
  {
    printf("Error: Unable to initialize ethernet communication\n");
    retval = Cortex_Exit();
    return 1;
  }
  
  // cortex frame rate //
  void *pResponse;
  int nBytes;
  printf("\n****** Cortex_FrameRate ******\n");
  retval = Cortex_Request("GetContextFrameRate", &pResponse, &nBytes);
  if (retval != RC_Okay)
    printf("ERROR, GetContextFrameRate\n");
  float *contextFrameRate = (float*) pResponse;
  printf("ContextFrameRate = %3.1f Hz\n", *contextFrameRate);


  // get name of bodies being tracked and its set of markers //
  printf("\n****** Cortex_GetBodyDefs ******\n");
  pBodyDefs = Cortex_GetBodyDefs();

  if (pBodyDefs == NULL) 
  {
    printf("Failed to get body defs\n");
  } 
  else 
  { 
    // cout << "total no of bodies tracked " << pBodyDefs->nBodyDefs << endl;
    // for(int iBody=0; iBody<pBodyDefs->nBodyDefs; iBody++)
    // {
    //   bodyMarkers.push_back(pBodyDefs->BodyDefs[iBody].nMarkers);
    //   sBodyDef* pBody = &pBodyDefs->BodyDefs[iBody];
    //   cout << "number of markers defined in body " << iBody+1 << " (\"" << pBody->szName << "\") : " << bodyMarkers.at(iBody) << endl;    
            
    //   for (int iMarker=0 ; iMarker<pBody->nMarkers; iMarker++)
    //   {
    //     cout << iMarker+1 << " " << pBody->szMarkerNames[iMarker] << endl;
    //   }

    // }
  }


  
  // live plot //
  // plot p;
  // vector<double> xm, xm1, xm2;


  printf("\n*** Starting live mode ***\n");
  retval = Cortex_Request("LiveMode", &pResponse, &nBytes);
  // double del = 0;  

  while(1)
  {
    pFrameOfData =  Cortex_GetCurrentFrame();  // Can POLL for the current frame. 

    // cout << "time delay from camera " << pFrameOfData->fDelay <<  "; cortex frame " << pFrameOfData->iFrame << endl;    
    
    // del+=pFrameOfData->fDelay;

    // recordingStat = pFrameOfData->RecordingStatus.bRecording;
    // if(recordingStat==0)
    // {  
    //     retval = Cortex_Request("StartRecording", &pResponse, &nBytes);
    //     if (retval != RC_Okay)
    //     printf("ERROR, not recording \n");
    // }

    for(int b = 0; b<pFrameOfData->nBodies; b++)
    {
      for(int m = 0; m<bodyMarkers.at(b); m++)
      {
        // cout << "body1 markers == " << (m+1) << endl <<
        //               " X: " << pFrameOfData->BodyData[b].Markers[m][0] << endl <<
        //               " Y: " << pFrameOfData->BodyData[b].Markers[m][1] << endl <<
        //               " Z: " << pFrameOfData->BodyData[b].Markers[m][2] << endl;

    // cout << pFrameOfData->BodyData[b].Markers[1][0] - pFrameOfData->BodyData[b].Markers[2][0] << endl;
    
    // xm.push_back(pFrameOfData->BodyData[b].Markers[1][0] - pFrameOfData->BodyData[b].Markers[2][0]);

    // xm1.push_back(pFrameOfData->BodyData[b].Markers[1][0]);
    // xm2.push_back(pFrameOfData->BodyData[b].Markers[3][0]);


      }
    }
    // cout << "size xm1 " << xm1.size() << endl; 
    // p.plot_data(xm1,xm2);

    // p.plot_data(xm);

    // usleep(1000000);
    if(pFrameOfData->iFrame == 1000)
    {
      // cout << "\ntotal delay from camera " << del << endl;
      // if(recordingStat!=0)
      // {        
      //   printf("\n****** stopped recording ******\n");
      //   Cortex_Request("StopRecording", &pResponse, &nBytes);
      // }
      
      printf("\n****** Paused live mode ... exiting Cortex ******\n");
      Cortex_FreeFrame(pFrameOfData);
      Cortex_Request("Pause", &pResponse, &nBytes);
      Cortex_Exit();
    }

  }
  return 0;
}