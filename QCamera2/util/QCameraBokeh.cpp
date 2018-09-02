/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define LOG_TAG "QCameraBokeh"
// System dependencies
#include <dlfcn.h>
#include <utils/Errors.h>
#include <stdio.h>
#include <stdlib.h>
// Camera dependencies
#include "QCameraBokeh.h"
#include "QCameraTrace.h"
extern "C" {
#include "mm_camera_dbg.h"
}

#include "dualcameraddm_wrapper.h"

const char* SCALE_CROP_ROTATION_FORMAT_STRING[] = {
        "Sensor Crop left = %d\n",
        "Sensor Crop top = %d\n",
        "Sensor Crop width = %d\n",
        "Sensor Crop height = %d\n",
        "Sensor ROI Map left = %d\n",
        "Sensor ROI Map top = %d\n",
        "Sensor ROI Map width = %d\n",
        "Sensor ROI Map height = %d\n",
        "CAMIF Crop left = %d\n",
        "CAMIF Crop top = %d\n",
        "CAMIF Crop width = %d\n",
        "CAMIF Crop height = %d\n",
        "CAMIF ROI Map left = %d\n",
        "CAMIF ROI Map top = %d\n",
        "CAMIF ROI Map width = %d\n",
        "CAMIF ROI Map height = %d\n",
        "ISP Crop left = %d\n",
        "ISP Crop top = %d\n",
        "ISP Crop width = %d\n",
        "ISP Crop height = %d\n",
        "ISP ROI Map left = %d\n",
        "ISP ROI Map top = %d\n",
        "ISP ROI Map width = %d\n",
        "ISP ROI Map height = %d\n",
        "CPP Crop left = %d\n",
        "CPP Crop top = %d\n",
        "CPP Crop width = %d\n",
        "CPP Crop height = %d\n",
        "CPP ROI Map left = %d\n",
        "CPP ROI Map top = %d\n",
        "CPP ROI Map width = %d\n",
        "CPP ROI Map height = %d\n",
        "Focal length Ratio = %f\n",
        "Current pipeline mirror flip setting = %d\n",
        "Current pipeline rotation setting = %d\n"
};

const char* CALIB_FMT_STRINGS[] = {
    "Calibration OTP format version = %d\n",
    "Main Native Sensor Resolution width = %dpx\n",
    "Main Native Sensor Resolution height = %dpx\n",
    "Main Calibration Resolution width = %dpx\n",
    "Main Calibration Resolution height = %dpx\n",
    "Main Focal length ratio = %f\n",
    "Aux Native Sensor Resolution width = %dpx\n",
    "Aux Native Sensor Resolution height = %dpx\n",
    "Aux Calibration Resolution width = %dpx\n",
    "Aux Calibration Resolution height = %dpx\n",
    "Aux Focal length ratio = %f\n",
    "Relative Rotation matrix [0] through [8] = %s\n",
    "Relative Geometric surface parameters [0] through [31] = %s\n",
    "Relative Principal point X axis offset (ox) = %fpx\n",
    "Relative Principal point Y axis offset (oy) = %fpx\n",
    "Relative position flag = %d\n",
    "Baseline distance = %fmm\n",
    "Main sensor mirror and flip setting = %d\n",
    "Aux sensor mirror and flip setting = %d\n",
    "Module orientation during calibration = %d\n",
    "Rotation flag = %d\n",
    "Main Normalized Focal length = %fpx\n",
    "Aux Normalized Focal length = %fpx"
};

#define SWAP(a, b) do { typeof(a) temp = a; a = b; b = temp; } while (0)
#define DIFF(x, y) ((x-y)>0?(x-y):0)
#define PMIN(a, b) ((b) > 0 ? ((a) < (b) ? (a) : (b)) : (a))

#define DUMP(fmt, args...)                           \
{                                                    \
    if (m_bDebug) {                                  \
        mDebugData.appendFormat(fmt, ##args);        \
    }                                                \
}

#define FDUMP(file, string, idx)                     \
{                                                    \
    if (m_bDebug) {                                  \
        dumpInputParams(file, string, idx);          \
    }                                                \
}


namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraBokeh
 *
 * DESCRIPTION: constructor of QCameraBokeh.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBokeh::QCameraBokeh() : QCameraHALPP()
{
    m_dlHandle = NULL;
    m_pCaps = NULL;
    memset(&mBokehData, 0, sizeof(mBokehData));
}

/*===========================================================================
 * FUNCTION   : ~QCameraBokeh
 *
 * DESCRIPTION: destructor of QCameraBokeh.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBokeh::~QCameraBokeh()
{
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of QCameraBokeh
 *
 * PARAMETERS :
 *   @bufNotifyCb    : call back function after HALPP process
 *   @getOutputCb   : call back function to request output buffer
 *   @pUserData      : Parent of HALPP, i.e. QCameraPostProc
 *   @pStaticParam  : holds dual camera calibration data in an array and its size
 *                       (expected size is 264 bytes)
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::init(
        halPPBufNotify bufNotifyCb,
        halPPGetOutput getOutputCb,
        void *pUserData,
        void *pStaticParam)
{
    LOGH("E");
    int32_t rc = NO_ERROR;


    QCameraHALPP::init(bufNotifyCb, getOutputCb, pUserData);

    m_pCaps = (cam_capability_t *)pStaticParam;

    /* we should load 3rd libs here, with dlopen/dlsym */
    doBokehInit();

    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: de initialization of QCameraBokeh
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::deinit()
{
    int32_t rc = NO_ERROR;
    LOGH("E");

    m_dlHandle = NULL;

    QCameraHALPP::deinit();
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: starting QCameraBokeh
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::start()
{
    int32_t rc = NO_ERROR;
    LOGH("E");

    rc = QCameraHALPP::start();

    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop QCameraBokeh
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::stop()
{
    int32_t rc = NO_ERROR;
    LOGH("E");

    rc = QCameraHALPP::stop();

    LOGH("X");
    return rc;
}


/*===========================================================================
 * FUNCTION   : feedInput
 *
 * DESCRIPTION: function to feed input data.
 *              Enqueue the frame index to inputQ if it is new frame
 *              Also, add the input image data to frame hash map
 *
 * PARAMETERS :
 *   @pInputData    : ptr to input data
 *   @bFeedOutput  : true if ready for feeding output
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::feedInput(qcamera_hal_pp_data_t *pInputData)
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (NULL != pInputData) {
        QCameraStream* pSnapshotStream = NULL;
        mm_camera_buf_def_t *pInputSnapshotBuf = getSnapshotBuf(pInputData, pSnapshotStream);
        if (pInputSnapshotBuf != NULL) {
            // Check for main and aux handles
            uint32_t mainHandle = get_main_camera_handle(
                    pInputData->src_reproc_frame->camera_handle);
            uint32_t auxHandle = get_aux_camera_handle(
                    pInputData->src_reproc_frame->camera_handle);

            LOGH("mainHandle = 0x%x, auxHandle = 0x%x", mainHandle, auxHandle);
            if ((!mainHandle) && (!auxHandle)) {
                // Both main and aux handles are not available
                // Return from here
                return BAD_VALUE;
            }
            if (mainHandle && (mBokehData.wide_input == NULL)) {
                mBokehData.wide_input = pInputData;
                LOGH("Update wide input");
            }
            else if (auxHandle && (mBokehData.tele_input == NULL)) {
                mBokehData.tele_input = pInputData;
                LOGH("Update tele input");
            }
            // request output buffer only if both wide and tele input data are recieved
            if ((mBokehData.tele_input != NULL) && (mBokehData.wide_input != NULL)) {
                m_halPPGetOutputCB(pInputSnapshotBuf->frame_idx, m_pHalPPMgr);
            }
        }
    } else {
        LOGE("pInput is NULL");
        rc = UNEXPECTED_NULL;
    }
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : feedOutput
 *
 * DESCRIPTION: function to feed output buffer and metadata
 *
 * PARAMETERS :
 *   @pOutput     : ptr to output data
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::feedOutput(qcamera_hal_pp_data_t *pOutputData)
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (NULL == pOutputData) {
        LOGE("Error! pOutput hal pp data is NULL");
        return BAD_VALUE;
    }
    if (mBokehData.tele_output != NULL) {
        releaseData(pOutputData);
        LOGE("Error!! Output data is not empty");
        return BAD_VALUE;
    }
    rc = getOutputBuffer(mBokehData.tele_input, pOutputData);
    if (rc == NO_ERROR) {
        LOGH("filling Tele output %d", rc);
        mBokehData.tele_output = pOutputData;
    }

    LOGH("X rc: %d", rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : process
 *
 * DESCRIPTION: Start Bokeh process
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::process()
{
    int32_t rc = NO_ERROR;

    /* dump in/out frames */
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.bokeh.dumpimg", prop, "0");
    int dumpimg = atoi(prop);

    LOGH("E");

    // Start the blending process when it is ready
    if (canProcess()) {
        LOGH("Start Bokeh processing");
        QCameraStream* pAuxStream = NULL;
        QCameraStream* pMainStream = NULL;

        mm_camera_buf_def_t *pAuxSnap = getSnapshotBuf(mBokehData.aux_input, pAuxStream);
        mm_camera_buf_def_t *pMainSnap = getSnapshotBuf(mBokehData.main_input, pMainStream);

        if (!pAuxStream || !pMainStream || !pAuxSnap || !pMainSnap) {
            releaseData(mBokehData.aux_input);
            releaseData(mBokehData.main_input);
            releaseData(mBokehData.bokeh_output);
            LOGE("Error!! Snapshot buffer/stream or depthmap not available");
            return BAD_VALUE;
        }
        mm_camera_super_buf_t *pOutputSuperBuf = mBokehData.tele_output->frame;
        mm_camera_buf_def_t *pOutputBuf = pOutputSuperBuf->bufs[0];

        // Use offset info from reproc stream
        cam_frame_len_offset_t frm_offset;
        memset(&frm_offset, 0, sizeof(frm_offset));
        pTeleStream->getFrameOffset(frm_offset);
        LOGH("Stream type:%d, stride:%d, scanline:%d, frame len:%d",
                pTeleStream->getMyType(),
                frm_offset.mp[0].stride, frm_offset.mp[0].scanline,
                frm_offset.frame_len);

        //Get input and output parameter
        bokeh_input_params_t inParams;
        getInputParams(inParams);

        rc = doBokehProcess(
                (const uint8_t *)pMainSnap->buffer,
                (const uint8_t *)pAuxSnap->buffer,
                inParams, (uint8_t *)pOutputBuf->buffer);

        uint8_t * pDepthMap = (uint8_t *)mBokehData.depth_output->frame->bufs[0]->buffer;

        LOGH("doing Bokeh process!!!");
        doBokehProcess(
                (const uint8_t *)pWideSnap->buffer,
                (const uint8_t *)pTeleSnap->buffer,
                inParams,
                (uint8_t *)pOutputBuf->buffer);

        if (dumpimg) {
            dumpYUVtoFile((uint8_t *)pTeleSnap->buffer, frm_offset,
                    pTeleSnap->frame_idx, "Tele");
            dumpYUVtoFile((uint8_t *)pWideSnap->buffer,  frm_offset,
                    pWideSnap->frame_idx,  "Wide");
            dumpYUVtoFile((uint8_t *)pOutputBuf->buffer, frm_offset,
                    pTeleSnap->frame_idx, "TeleBokeh");
        }

        // Invalidate input buffer
        QCameraMemory *pMem = NULL;
        pMem = (QCameraMemory *)pTeleSnap->mem_info;
        pMem->invalidateCache(pTeleSnap->buf_idx);
        pMem = (QCameraMemory *)pWideSnap->mem_info;
        pMem->invalidateCache(pWideSnap->buf_idx);
        // Clean and invalidate output buffer
        mBokehData.tele_output->snapshot_heap->cleanInvalidateCache(0);

        // Callback Manager to notify output buffer and return input buffers
        LOGH("notifying Bokeh output");
        m_halPPBufNotifyCB(mBokehData.bokeh_output, m_pHalPPMgr);
        LOGH("CB for main input");
        m_halPPBufNotifyCB(mBokehData.main_input, m_pHalPPMgr);
        LOGH("CB for depth map");
        m_halPPBufNotifyCB(mBokehData.depth_output, m_pHalPPMgr);
        LOGH("CB for aux input");
        m_halPPBufNotifyCB(mBokehData.aux_input, m_pHalPPMgr);

        // Once process is complete, reset context data
        // Post proc would take care of releasing the data
        memset(&mBokehData, 0, sizeof(mBokehData));


    }
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : canProcess
 *
 * DESCRIPTION: function to release internal resources
 * RETURN     : If Bokeh module can process
 *==========================================================================*/
bool QCameraBokeh::canProcess()
{
    LOGH("E");
    bool ready = false;
    //Check if we have all input and output buffers
    if (mBokehData.wide_input && mBokehData.tele_input && mBokehData.tele_output) {
        ready = true;
        LOGH("ready: %d", ready);
    }
    LOGH("X");
    return ready;
}

/*===========================================================================
 * FUNCTION   : getInputParams
 *
 * DESCRIPTION: Helper function to get input params from input metadata
 *==========================================================================*/
void QCameraBokeh::getInputParams(bokeh_input_params_t& inParams)
{
    LOGH("E");
    memset(&inParams, 0, sizeof(bokeh_input_params_t));
    QCameraStream* pTeleStream = NULL;
    QCameraStream* pWideStream = NULL;
    //QCameraStream* pTeleMetaStream  = NULL;
    //QCameraStream* pWideMetaStream  = NULL;

    mm_camera_buf_def_t *pTeleSnap = getSnapshotBuf(mBokehData.tele_input, pTeleStream);
    mm_camera_buf_def_t *pWideSnap = getSnapshotBuf(mBokehData.wide_input, pWideStream);
    if (!pTeleSnap || !pWideSnap) {
        LOGH("NULL pointer pTeleSnap: %p, pWideSnap: %p", pTeleSnap, pWideSnap);
        return;
    }

    //mm_camera_buf_def_t *pTeleMeta = getMetadataBuf(mBokehData.tele_input, pTeleMetaStream);
    //mm_camera_buf_def_t *pWideMeta = getMetadataBuf(mBokehData.wide_input, pWideMetaStream);
    //metadata_buffer_t *pTeleMetaBuf = (metadata_buffer_t *)pTeleMeta->buffer;
    //metadata_buffer_t *pWideMetaBuf = (metadata_buffer_t *)pWideMeta->buffer;

    // Wide frame size
    cam_frame_len_offset_t offset;
    pWideStream->getFrameOffset(offset);
    inParams.wide.width     = offset.mp[0].width;
    inParams.wide.height    = offset.mp[0].height;
    inParams.wide.stride    = offset.mp[0].stride;
    inParams.wide.scanline  = offset.mp[0].scanline;
    inParams.wide.frame_len = offset.frame_len;
    LOGH("Wide width: %d height: %d stride:%d, scanline:%d frame_len: %d",
            inParams.wide.width, inParams.wide.height,
            inParams.wide.stride, inParams.wide.scanline,
            inParams.wide.frame_len);

    // Tele frame size
    pTeleStream->getFrameOffset(offset);
    inParams.tele.width     = offset.mp[0].width;
    inParams.tele.height    = offset.mp[0].height;
    inParams.tele.stride    = offset.mp[0].stride;
    inParams.tele.scanline  = offset.mp[0].scanline;
    inParams.tele.frame_len = offset.frame_len;

    LOGH("Tele width: %d height: %d stride:%d, scanline:%d",
            inParams.tele.width, inParams.tele.height,
            inParams.tele.stride, inParams.tele.scanline,
            inParams.tele.frame_len);

    dumpInputParams(inParams);

    memset(&inParams.zoomROI, 0, sizeof(inParams.zoomROI));
    IF_META_AVAILABLE(cam_crop_data_t, crop, CAM_INTF_META_CROP_DATA, pMainMetaBuf) {
        QCameraStream* pSnapStream = NULL;
        qcamera_hal_pp_data_t tempData;
        tempData.frame = mBokehData.main_input->src_reproc_frame;
        getSnapshotBuf(&tempData, pSnapStream);
        if (pSnapStream != NULL) {
            for (int j = 0; j < crop->num_of_streams; j++) {
                    cam_rect_t streamCrop = crop->crop_info[j].crop;
                    if (pSnapStream->getMyServerID() == crop->crop_info[j].stream_id) {
                        inParams.zoomROI = streamCrop;
                        DUMP("Zoom ROI : (%d, %d, %d, %d)",
                                streamCrop.left, streamCrop.top,
                                streamCrop.width, streamCrop.height);
                        break;
                    }
            }
        }
    }

    return;
}


int32_t QCameraBokeh::doBokehInit()
{
    LOGH("E");
    int rc = NO_ERROR;
    LOGH("E");
    return rc;
}

int32_t QCameraBokeh::doBokehProcess(
        const uint8_t* pMain,
        const uint8_t* pAux,
        bokeh_input_params_t &inParams,
        uint8_t* pOut)
{
    LOGD(":E");
    ATRACE_BEGIN("doBokehProcess");
    int32_t rc = NO_ERROR;
    QCameraStream* pStream = NULL;
    uint32_t focusX,focusY;
    qrcp::DualCameraDDMEffects *effectObj = NULL;
    qrcp::DualCameraDDMEffects::EffectType type = qrcp::DualCameraDDMEffects::REFOCUS_CIRCLE;
    char prop[PROPERTY_VALUE_MAX];

    //1. get depth map size from lib
    cam_dimension_t dmSize;
    qrcp::getDepthMapSize(inParams.main.width, inParams.main.height,
            dmSize.width, dmSize.height);
    unsigned int depthStride = dmSize.width;
    uint32_t depthLen = dmSize.width * dmSize.height;
    inParams.depth.offset.num_planes = 1;
    inParams.depth.offset.mp[0].offset = 0;
    inParams.depth.offset.mp[0].width = inParams.depth.offset.mp[0].stride =
            inParams.depth.width = inParams.depth.stride = dmSize.width;
    inParams.depth.offset.mp[0].height = inParams.depth.offset.mp[0].scanline =
            inParams.depth.height = inParams.depth.scanline = dmSize.height;
    inParams.depth.frame_len = inParams.depth.offset.frame_len =
            inParams.depth.offset.mp[0].len = depthLen;
    rc = allocateDepthBuf(inParams.depth);
    if(rc != NO_ERROR)
    {
        ATRACE_END();
        LOGE("ERROR: Failed to allocate depth buffer, error no:%d X",rc);
        return rc;
    }
    uint8_t * pDepthMap = (uint8_t *)mBokehData.depth_output->frame->bufs[0]->buffer;

    DUMP("\nDepth map W %d H %d ", dmSize.width, dmSize.height);

    //2. generate depth map
    const uint8_t* primaryY = pMain;
    uint32_t main_uv_offset = inParams.main.offset.mp[0].len;
    const uint8_t* primaryVU = pMain + main_uv_offset;
    unsigned int primaryWidth = inParams.main.width;
    unsigned int primaryHeight = inParams.main.height;
    unsigned int primaryStrideY = inParams.main.stride;
    unsigned int primaryStrideVU = inParams.main.offset.mp[1].stride;

    const uint8_t* auxiliaryY = pAux;
    uint32_t aux_uv_offset = inParams.aux.offset.mp[0].len;
    const uint8_t* auxiliaryVU = pAux + aux_uv_offset;
    unsigned int auxiliaryWidth = inParams.aux.width;
    unsigned int auxiliaryHeight = inParams.aux.height;
    unsigned int auxiliaryStrideY = inParams.aux.stride;
    unsigned int auxiliaryStrideVU = inParams.aux.offset.mp[1].stride;

    cam_rect_t goodRoi = {0,0,0,0};
    const float focalLengthPrimaryCamera =
        MAX(m_pCaps->main_cam_cap->focal_length, m_pCaps->aux_cam_cap->focal_length);

    uint8_t dualCamConfig = DUAL_CAM_CONFIG;
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.ddm.config", prop, "");
    if (strlen(prop) > 0) {
        dualCamConfig = atoi(prop);
    }
    qrcp::SensorConfiguration config = (qrcp::SensorConfiguration) dualCamConfig;

    qrcp::DepthMapMode ddmMode = qrcp::DepthMapMode::ENHANCED_MODE;
    if (QCameraCommon::is_target_SDM630())
        ddmMode = qrcp::DepthMapMode::NORMAL_MODE;

    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.bokeh.ddmmode", prop, "");
    if (strlen(prop) > 0) {
        if (!strcmp(prop, "normal"))
            ddmMode = qrcp::DepthMapMode::NORMAL_MODE;
        else if (!strcmp(prop, "enhanced"))
            ddmMode = qrcp::DepthMapMode::ENHANCED_MODE;
    }

    DUMP("\nDepthStride = %d \nfocalLengthPrimaryCamera = %f \n"
            "SensorConfiguration = %d", depthStride, focalLengthPrimaryCamera, config);
    DUMP("\nDepthmap mode = %s ", (ddmMode == qrcp::DepthMapMode::NORMAL_MODE) ?
            "normal" : "enhanced");

    LOGI("[KPI Perf]: PROFILE_BOKEH_PROCESS : E");
    qrcp::DDMWrapperStatus status = qrcp::dualCameraGenerateDDM(
            primaryY, primaryVU, primaryWidth, primaryHeight,
            primaryStrideY, primaryStrideVU,
            auxiliaryY, auxiliaryVU, auxiliaryWidth, auxiliaryHeight,
            auxiliaryStrideY, auxiliaryStrideVU,
            pDepthMap, depthStride,
            goodRoi.left, goodRoi.top, goodRoi.width,goodRoi.height,
            inParams.sMainReprocessInfo.string(), inParams.sAuxReprocessInfo.string(),
            inParams.sCalibData.string(), focalLengthPrimaryCamera, config, ddmMode);

    if (!status.ok()) {
        LOGE("depth map generation failed: %s, errorcode %d",
                status.getErrorMessage().c_str(), status.getErrorCode());
        rc = BAD_VALUE;
        goto done;
    }

    LOGH("Depth map generated successfully");
    DUMP("\nGood ROI : (%d, %d, %d, %d)",
            goodRoi.left, goodRoi.top, goodRoi.width,goodRoi.height);

    //3. Bokeh processing using above depth map
    if ((inParams.afROIMap.width != 0) && (inParams.afROI.width != 0) &&
            (inParams.afROIMap.height != 0) && (inParams.afROI.height != 0)) {
        //get center of AF ROI and scale it from sensor coordinates (ROImap) to goodROI.
        focusX = inParams.afROI.left + inParams.afROI.width/2;
        focusX = focusX * primaryWidth / inParams.afROIMap.width;
        focusX = (focusX - goodRoi.left) * goodRoi.width / primaryWidth;
        focusY = inParams.afROI.top + inParams.afROI.height/2;
        focusY = focusY * primaryHeight / inParams.afROIMap.height;
        focusY = (focusY - goodRoi.top) * goodRoi.height / primaryHeight;
    } else {
        // Copy Tele if both sizes are not same
        // Y
        memcpy(pOut, pTele, inParams.tele.stride * inParams.tele.scanline);
        // UV
        uint32_t uv_offset = inParams.tele.stride * inParams.tele.scanline;
        memcpy(pOut  + uv_offset, pTele + uv_offset,
                inParams.tele.stride * (inParams.tele.scanline / 2));
    }

    LOGI("[KPI Perf]: PROFILE_BOKEH_PROCESS : X");

    //Bokeh output size will be goodROI.width X goodROI.height.
    //set stream crop info so that jpeg will crop and upscale to original image size.
    //Take into account any zoom applied.
    getSnapshotBuf(mBokehData.bokeh_output, pStream);
    cam_rect_t bokeh_out_dim;
    bokeh_out_dim.top = DIFF(inParams.zoomROI.top, goodRoi.top);
    bokeh_out_dim.left = DIFF(inParams.zoomROI.left, goodRoi.left);
    bokeh_out_dim.width = PAD_TO_SIZE(PMIN(goodRoi.width, inParams.zoomROI.width), CAM_PAD_TO_2);
    bokeh_out_dim.height = PAD_TO_SIZE(PMIN(goodRoi.height, inParams.zoomROI.height), CAM_PAD_TO_2);
    DUMP("\nBokeh Crop Left %d Top %d Width %d Height %d ", bokeh_out_dim.left, bokeh_out_dim.top,
            bokeh_out_dim.width,bokeh_out_dim.height);
    if (pStream != NULL) {
        pStream->setCropInfo(bokeh_out_dim);
    }

    //apply zoom, if any, on depth map
    cam_rect_t depthCrop;
    depthCrop.top =  inParams.zoomROI.top * dmSize.height / primaryHeight;
    depthCrop.left = inParams.zoomROI.left * dmSize.width / primaryWidth;
    depthCrop.width =
            PAD_TO_SIZE((inParams.zoomROI.width * dmSize.width / primaryWidth), CAM_PAD_TO_2);
    depthCrop.height =
            PAD_TO_SIZE((inParams.zoomROI.height * dmSize.height / primaryHeight), CAM_PAD_TO_2);
    getSnapshotBuf(mBokehData.aux_input, pStream);
    if (pStream != NULL) {
        pStream->setCropInfo(depthCrop);
    }
    DUMP("\nDepth Crop Left %d Top %d Width %d Height %d ", depthCrop.left, depthCrop.top,
            depthCrop.width,depthCrop.height);

    //modify bokeh offset
    inParams.bokehOut.width = inParams.bokehOut.offset.mp[0].width = goodRoi.width;
    inParams.bokehOut.height = inParams.bokehOut.offset.mp[0].height = goodRoi.height;
    inParams.bokehOut.offset.mp[1].width = goodRoi.width;
    inParams.bokehOut.offset.mp[1].height = goodRoi.height/2;

done:
    if (effectObj) {
        delete effectObj;
    }
    ATRACE_END();
    LOGD("X");
    return rc;
}

void QCameraBokeh::dumpYUVtoFile(
        const uint8_t* pBuf,
        cam_frame_len_offset_t offset,
        uint32_t idx,
        const char* name_prefix)
{
    LOGH("E.");
    char filename[256];

    snprintf(filename, sizeof(filename), QCAMERA_DUMP_FRM_LOCATION"%s_%dx%d_%d.yuv",
                name_prefix, offset.mp[0].stride, offset.mp[0].scanline, idx);

    QCameraHALPP::dumpYUVtoFile(pBuf,(const char*)filename, offset.frame_len);

    LOGH("X.");
}

void QCameraBokeh::dumpInputParams(const bokeh_input_params_t& p)
{
    LOGH("E");

    const cam_frame_size_t* s = NULL;

    s = &p.wide;
    LOGH("wide frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    s = &p.tele;
    LOGH("wide frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    calibData.appendFormat(CALIB_FMT_STRINGS[0], calib_data.calibration_format_version);
    calibData.appendFormat(CALIB_FMT_STRINGS[1],
            calib_data.main_cam_specific_calibration.native_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[2],
            calib_data.main_cam_specific_calibration.native_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[3],
            calib_data.main_cam_specific_calibration.calibration_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[4],
            calib_data.main_cam_specific_calibration.calibration_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[5],
            calib_data.main_cam_specific_calibration.focal_length_ratio);

    calibData.appendFormat(CALIB_FMT_STRINGS[6],
            calib_data.aux_cam_specific_calibration.native_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[7],
            calib_data.aux_cam_specific_calibration.native_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[8],
            calib_data.aux_cam_specific_calibration.calibration_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[9],
            calib_data.aux_cam_specific_calibration.calibration_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[10],
            calib_data.aux_cam_specific_calibration.focal_length_ratio);

    calibData.appendFormat(CALIB_FMT_STRINGS[11],
            buildCommaSeparatedString(calib_data.relative_rotation_matrix,
            RELCAM_CALIB_ROT_MATRIX_MAX));
    calibData.appendFormat(CALIB_FMT_STRINGS[12],
            buildCommaSeparatedString(calib_data.relative_geometric_surface_parameters,
            RELCAM_CALIB_SURFACE_PARMS_MAX));

    calibData.appendFormat(CALIB_FMT_STRINGS[13], calib_data.relative_principle_point_x_offset);
    calibData.appendFormat(CALIB_FMT_STRINGS[14], calib_data.relative_principle_point_y_offset);
    calibData.appendFormat(CALIB_FMT_STRINGS[15], calib_data.relative_position_flag);
    calibData.appendFormat(CALIB_FMT_STRINGS[16], calib_data.relative_baseline_distance);
    calibData.appendFormat(CALIB_FMT_STRINGS[17], calib_data.main_sensor_mirror_flip_setting);
    calibData.appendFormat(CALIB_FMT_STRINGS[18], calib_data.aux_sensor_mirror_flip_setting);
    calibData.appendFormat(CALIB_FMT_STRINGS[19], calib_data.module_orientation_during_calibration);
    calibData.appendFormat(CALIB_FMT_STRINGS[20], calib_data.rotation_flag);
    calibData.appendFormat(CALIB_FMT_STRINGS[21],
            calib_data.main_cam_specific_calibration.normalized_focal_length);
    calibData.appendFormat(CALIB_FMT_STRINGS[22],
            calib_data.aux_cam_specific_calibration.normalized_focal_length);

    return calibData;
}

void QCameraBokeh::dumpInputParams(const char* file, String8 str, uint32_t idx)
{
    char filename[256];
    snprintf(filename, sizeof(filename),
            QCAMERA_DUMP_FRM_LOCATION"%s_%d.txt",file, idx);

    int file_fd = open(filename, O_RDWR | O_CREAT, 0777);
    if (file_fd > 0) {
        fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        write(file_fd, str.string(), str.size());
        close(file_fd);
    }
}

int32_t QCameraBokeh::allocateDepthBuf(cam_frame_size_t depthSize)
{
    int32_t rc = NO_ERROR;
    QCameraStream* pSnapshotStream = NULL;
    mm_camera_super_buf_t *pInputFrame = mBokehData.aux_input->frame;
    mm_camera_buf_def_t *pInputSnapshotBuf = getSnapshotBuf(mBokehData.aux_input, pSnapshotStream);

    if(!pInputSnapshotBuf || !pSnapshotStream)
    {
        releaseData(mBokehData.aux_input);
        LOGE("Error!! Snapshot buffer/stream or depthmap not available");
        return BAD_VALUE;
    }

    qcamera_hal_pp_data_t *output_data =
            (qcamera_hal_pp_data_t*) malloc(sizeof(qcamera_hal_pp_data_t));
    if (output_data == NULL) {
        LOGE("No memory for qcamera_hal_pp_data_t output data");
        return NO_MEMORY;
    }
    memset(output_data, 0, sizeof(qcamera_hal_pp_data_t));
    mm_camera_super_buf_t* output_frame =
            (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (output_frame == NULL) {
        LOGE("No memory for mm_camera_super_buf_t frame");
        free(output_data);
        return NO_MEMORY;
    }
    memset(output_frame, 0, sizeof(mm_camera_super_buf_t));
    output_data->frame = output_frame;

    // Copy main input frame info to output frame
    output_frame->camera_handle = pInputFrame->camera_handle;
    output_frame->ch_id = pInputFrame->ch_id;
    output_frame->num_bufs = 1;//depth

    output_data->bufs =
            (mm_camera_buf_def_t *)malloc(sizeof(mm_camera_buf_def_t));
    if (output_data->bufs == NULL) {
        LOGE("No memory for output_data->bufs");
        free(output_frame);
        free(output_data);
        return NO_MEMORY;
    }
    memset(output_data->bufs, 0, sizeof(mm_camera_buf_def_t));
    output_data->halPPAllocatedBuf = true;
    output_data->snapshot_heap = new QCameraHeapMemory(QCAMERA_ION_USE_CACHE);
    if (output_data->snapshot_heap == NULL) {
        LOGE("Unable to new heap memory obj for image buf");
        free(output_frame);
        free(output_data->bufs);
        free(output_data);
        return NO_MEMORY;
    }

    uint32_t depthLen = depthSize.width * depthSize.height;
    rc = output_data->snapshot_heap->allocate(1, depthLen);
    if (rc < 0) {
        LOGE("Unable to allocate heap memory for image buf");
        releaseData(output_data);
        return NO_MEMORY;
    }

    mm_camera_buf_def_t *pOutputBufDefs = output_data->bufs;
    output_frame->bufs[0] = &pOutputBufDefs[0];
    memcpy(&pOutputBufDefs[0], pInputSnapshotBuf, sizeof(mm_camera_buf_def_t));
    output_data->snapshot_heap->getBufDef(depthSize.offset, pOutputBufDefs[0], 0);

    mBokehData.depth_output = output_data;

    //Modify Aux stream properties for depth map
    //set depth map dimensions
    cam_dimension_t depth_dim;
    depth_dim.width = depthSize.width;
    depth_dim.height = depthSize.height;
    pSnapshotStream->setFrameDimension(depth_dim);
    //set depth map offset info
    pSnapshotStream->setFrameOffset(depthSize.offset);
    //set depth map format
    pSnapshotStream->setFormat(CAM_FORMAT_Y_ONLY);
    return rc;
}

} // namespace qcamera
