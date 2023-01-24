#include "vpi_feature_tracker_fast.h"

int FeatureTracker::n_id = 0;
int filecount = 0;

bool inBorder(const cv::Point2f &pt)
{
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector1(vector<cv::Point2f> &v, vector<uchar> status,VPIArray correspondingVPIarr=NULL)
{
    int j = 0;
    VPIArrayData arrData;
    VPIKeypoint *arrPoints;
    bool flag = false;
    if (correspondingVPIarr !=NULL){
        CHECK_STATUS(vpiArrayLock(correspondingVPIarr, VPI_LOCK_READ_WRITE, &arrData));
        arrPoints = (VPIKeypoint*)arrData.data;
        flag = true;
    }
    for (int i = 0; i < int(v.size()); i++)
        if (status[i]){
            v[j] = v[i];
            if (flag)
            {
                arrPoints[j].x = arrPoints[i].x;
                arrPoints[j].y = arrPoints[i].y;
            }
            j = j+1;
        }
    v.resize(j);
    if (flag){
        *arrData.sizePointer = j;
        vpiArrayUnlock(correspondingVPIarr);
    }
    // std::cerr<<"arr Size:"<<j<<std::endl;
}
// void outputVPIKeypoints(VPIArray src)
// {
//     VPIArrayData srcdata;
//     CHECK_STATUS(vpiArrayLock(src,VPI_LOCK_READ,&srcdata));
//     const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.data;
//     int totKeypoints = *srcdata.sizePointer;
//     std::cerr<<"output keypoint in VPI array"<<std::endl;
//     for (int i = 0;i<totKeypoints;i++){
//                 std::cerr<<"("<<srcPoints[i].x<<","<<srcPoints[i].y<<")";
//         }
//         std::cerr<<std::endl;
//         CHECK_STATUS(vpiArrayUnlock(src));
// }

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void VPIKeyPointArr_to_cvPointVec(VPIArray &src, vector<cv::Point2f> &dst){
    //std::cerr<<"vpi keypoints arr to cv point vec"<<std::endl;
    VPIArrayData srcdata;
    CHECK_STATUS(vpiArrayLock(src,VPI_LOCK_READ,&srcdata));
    // std::cerr<<"finish lock"<<std::endl;
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.data;
    // std::cerr<<"finish VPIKEYPOINT"<<std::endl;
    int totKeypoints = *srcdata.sizePointer;
    //std::cerr<<"finish totkeypoints:"<<totKeypoints<<std::endl;
    dst.resize(totKeypoints);
    for (int i =0;i<totKeypoints;i++)
    {   
        // std::cerr<<"i="<<i<<std::endl;
        cv::Point2f srcPoint{srcPoints[i].x,srcPoints[i].y};
        // std::cerr<<srcPoint.x<<","<<srcPoint.y<<std::endl;
        // std::cerr<<dst.size()<<std::endl;
        dst[i] = srcPoint;
        // std::cerr<<"finish write"<<std::endl;
    }
    // std::cerr<<"finish copying"<<std::endl;
    // std::cerr<<"finish resize"<<std::endl;
    CHECK_STATUS(vpiArrayUnlock(src));
    //std::cerr<<"finish vpi keypoints arr to cv point vec"<<std::endl;
}
void VPIstatus_to_cvStatus(VPIArray &src, vector<uchar> &dst){
    VPIArrayData srcdata;
    CHECK_STATUS(vpiArrayLock(src,VPI_LOCK_READ, &srcdata));
    const uint8_t *srcStatus = (uint8_t*) srcdata.data;
    int size = *srcdata.sizePointer;
    dst.resize(size);
    for (int i=0;i<size;i++){
        dst[i] = 1-srcStatus[i];
        // if (srcStatus[i] == 0){
        //     dst[i] = 1;
        // }
        // else{
        //     dst[i] = 0;
        // }
            //dst[i] = srcStatus[i];
            //std::cerr<<int(dst[i]);
    }
    //std::cerr<<std::endl;
    
    CHECK_STATUS(vpiArrayUnlock(src));
}

vector<cv::Point2f> pickPtsByQuatTree(VPIArray src, VPIArray scores, std::size_t amount)
{   
    // std::cerr<<"creating array data"<<std::endl;
    VPIArrayData srcdata;
    VPIArrayData scoresdata;
    CHECK_STATUS(vpiArrayLock(src,VPI_LOCK_READ,&srcdata));
    CHECK_STATUS(vpiArrayLock(scores,VPI_LOCK_READ,&scoresdata));
    // std::cerr<<"finish lock"<<std::endl;
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.data;
    // std::cerr<<"1"<<std::endl;
    const float * ptsScores = (float *)scoresdata.data;
    // std::cerr<<"2"<<std::endl;
    int totKeypoints = *srcdata.sizePointer;
    // std::cerr<<"init"<<std::endl;
    vector<cv::Point2f> tmp(totKeypoints);
    vector<float> tmpscores(totKeypoints);
    for (int i=0;i<totKeypoints;i++){
        cv::Point2f srcPoint{srcPoints[i].x,srcPoints[i].y};
        tmp[i] = srcPoint;
        tmpscores[i] = ptsScores[i];
    }
    CHECK_STATUS(vpiArrayUnlock(src));
    CHECK_STATUS(vpiArrayUnlock(scores));
    //  std::cerr<<"finish init"<<std::endl;
    if (totKeypoints<amount){
        return tmp;
    }
    // std::cerr<<"creating the first node"<<std::endl;
    node firstNode;
    firstNode.beginIdx = 0;
    firstNode.endIdx = totKeypoints; //next one of the last index
    firstNode.minCol = 0;
    firstNode.minRow = 0;
    firstNode.maxCol = COL;
    firstNode.maxRow = ROW;
    vector<node> nodes;
    nodes.push_back(firstNode);
    while (nodes.size()<amount){
        // std::cerr<<"amount:"<<amount<<","<<"current node number:"<<nodes.size()<<std::endl;
        nodes = splitNode(tmp,tmpscores,nodes);
    }
    //pick the points in each node with the highest score
    vector<pair<cv::Point2f,float>> n_pts_score;
    for (auto &it : nodes){
        int maxIdx = it.beginIdx;
        for (int i = it.beginIdx;i<it.endIdx;i++){
            if (tmpscores[i]>tmpscores[maxIdx]){
                maxIdx = i;
            }
        }
        n_pts_score.push_back(std::make_pair(tmp[maxIdx],tmpscores[maxIdx]));
    }
    std::partial_sort(n_pts_score.begin(),n_pts_score.begin()+amount,n_pts_score.end(),[](const pair<cv::Point2f, float> &a, const pair<cv::Point2f, float> &b){
        return a.second>=b.second;
    });
    vector<cv::Point2f> n_pts;
    for (int i=0;i<amount;i++){
        n_pts.push_back(n_pts_score[i].first);
    }
    return n_pts;
}

vector<node> splitNode(vector<cv::Point2f> &v, vector<float> &scores, vector<node> &info){
    // for each node, split it into four child nodes
    vector<node> newNodes;
    while (info.size()>0){
        node pNode = info[info.size()-1];
        info.pop_back();
        if (pNode.endIdx-pNode.beginIdx <=1){
            newNodes.emplace_back(pNode);
            continue;
        }
        vector<cv::Point2f> ul;
        vector<float> score_ul;
        vector<cv::Point2f> ur;
        vector<float> score_ur;
        vector<cv::Point2f> bl;
        vector<float> score_bl;
        vector<cv::Point2f> br;
        vector<float> score_br;
        int splitCol = (pNode.minCol + pNode.maxCol)/2;
        int splitRow = (pNode.minRow + pNode.maxRow)/2;
        for (int i = pNode.beginIdx;i<pNode.endIdx;i++){
            if (v[i].y <=splitRow){
                if (v[i].x<=splitCol){
                   ul.emplace_back(v[i]);
                   score_ul.emplace_back(scores[i]); 
                }
                else {
                    ur.emplace_back(v[i]);
                    score_ur.emplace_back(scores[i]);
                }
            }
            else{
                if(v[i].x<=splitCol){
                    bl.emplace_back(v[i]);
                    score_bl.emplace_back(scores[i]);
                }
                else{
                    br.emplace_back(v[i]);
                    score_br.emplace_back(scores[i]);
                }
            }
        }
        node ulnode;
        ulnode.beginIdx = pNode.beginIdx;
        ulnode.endIdx = ulnode.beginIdx + ul.size();
        // std::cerr<<"ul node size:"<<ul.size()<<std::endl;
        // std::cerr<<"["<<ulnode.beginIdx<<","<<ulnode.endIdx<<")"<<std::endl;
        if (ul.size() != 0){
            ulnode.minRow = pNode.minRow;
            ulnode.minCol = pNode.minCol;
            ulnode.maxRow = splitRow;
            ulnode.maxCol = splitCol;
            newNodes.emplace_back(ulnode);
            //rearrange node sequence
            for (int i = ulnode.beginIdx;i<ulnode.endIdx;i++){
                v[i] = ul[i-ulnode.beginIdx];
                scores[i] = score_ul[i-ulnode.beginIdx];
            }
        }
        node urnode;
        urnode.beginIdx = ulnode.endIdx;
        urnode.endIdx = urnode.beginIdx + ur.size();
        // std::cerr<<"ur node size:"<<ur.size()<<std::endl;
        // std::cerr<<"["<<urnode.beginIdx<<","<<urnode.endIdx<<")"<<std::endl;
        if (ur.size() != 0){
            urnode.minRow = pNode.minRow;
            urnode.minCol = splitCol;
            urnode.maxRow = splitRow;
            urnode.maxCol = pNode.maxCol;
            newNodes.emplace_back(urnode);
            for (int i = urnode.beginIdx;i<urnode.endIdx;i++){
                v[i] = ur[i-urnode.beginIdx];
                scores[i] = score_ur[i-urnode.beginIdx];
            }
        }
        node blnode;
        blnode.beginIdx = urnode.endIdx;
        blnode.endIdx = blnode.beginIdx + bl.size();
        //         std::cerr<<"bl node size:"<<bl.size()<<std::endl;
        // std::cerr<<"["<<blnode.beginIdx<<","<<blnode.endIdx<<")"<<std::endl;
        if (bl.size() != 0){
            blnode.minRow = splitRow;
            blnode.minCol = pNode.minCol;
            blnode.maxRow = pNode.maxRow;
            blnode.maxCol = splitCol;
            newNodes.emplace_back(blnode);
            for (int i = blnode.beginIdx;i<blnode.endIdx;i++){
                v[i] = bl[i-blnode.beginIdx];
                scores[i] = score_bl[i-blnode.beginIdx];
            }
        }
        node brnode;
        brnode.beginIdx = blnode.endIdx;
        brnode.endIdx = pNode.endIdx;//brnode.beginIdx + br.size()
        // std::cerr<<"br node size:"<<br.size()<<std::endl;
        // std::cerr<<"["<<brnode.beginIdx<<","<<brnode.endIdx<<")"<<std::endl;
        if (br.size() != 0){
            brnode.minCol = splitCol;
            brnode.minRow = splitRow;
            brnode.maxCol = pNode.maxCol;
            brnode.maxRow = pNode.maxRow;
            newNodes.emplace_back(brnode);
            for (int i = brnode.beginIdx;i<brnode.endIdx;i++){
                v[i] = br[i-brnode.beginIdx];
                scores[i] = score_br[i-brnode.beginIdx];
            }
        }
    }
    return newNodes;
}



FeatureTracker::FeatureTracker()
{
    backend = VPI_BACKEND_CUDA;
    pyrLevel = 5;
    round = 0;
}
void SortKeypoints(VPIArray keypoints, VPIArray scores, std::size_t max){
    VPIArrayData ptsData, scoresData;
    CHECK_STATUS(vpiArrayLock(keypoints, VPI_LOCK_READ_WRITE, &ptsData));
    CHECK_STATUS(vpiArrayLock(scores, VPI_LOCK_READ_WRITE, &scoresData));
    std::vector<int> indices(*ptsData.sizePointer);
    std::iota(indices.begin(), indices.end(), 0);
    max = max<indices.size() ? max:indices.size();
    // only need to sort the first 'max' keypoints
    //report error when the actural size smaller than bound
    std::partial_sort(indices.begin(), indices.begin() + max, indices.end(), [&scoresData](int a, int b) {
        uint32_t *score = reinterpret_cast<uint32_t *>(scoresData.data);
        return score[a] >= score[b]; // decreasing score order
    });
    // keep the only 'max' indexes.
    indices.resize(std::min<size_t>(indices.size(), max));

    VPIKeypoint *kptData = reinterpret_cast<VPIKeypoint *>(ptsData.data);
    std::vector<VPIKeypoint> kpt;
    std::transform(indices.begin(), indices.end(), std::back_inserter(kpt),
                   [kptData](int idx) { return kptData[idx]; });
    std::copy(kpt.begin(), kpt.end(), kptData);

    // update keypoint array size.
    *ptsData.sizePointer = kpt.size();
    vpiArrayUnlock(scores);
    vpiArrayUnlock(keypoints);
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    //add arrForwPts clear finished
    vpiArrayDestroy(arrForwPts);
    CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts));
    ids.clear();
    track_cnt.clear();
    VPIArrayData forwArrData;
    VPIKeypoint *forwArrPoints;
    CHECK_STATUS(vpiArrayLock(arrForwPts, VPI_LOCK_READ_WRITE, &forwArrData));
    forwArrPoints = (VPIKeypoint*)forwArrData.data;
    int i = 0;
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            forwArrPoints[i].x = it.second.first.x;
            forwArrPoints[i].y = it.second.first.y;
            //add addForwPts finished
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            i = i+1;
        }
    }
    *forwArrData.sizePointer = i;
    vpiArrayUnlock(arrForwPts);
    //     for (auto &i:track_cnt) std::cerr<<i<<",";
    // std::cerr<<std::endl;
}

void FeatureTracker::addPoints()
{   
    VPIArrayData forwArrData;
    CHECK_STATUS(vpiArrayLock(arrForwPts, VPI_LOCK_READ_WRITE, &forwArrData));
    VPIKeypoint *forwArrPoints = (VPIKeypoint*)forwArrData.data;
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        //add arrforwpts
        int32_t end_idx= *forwArrData.sizePointer;
        *forwArrData.sizePointer = *forwArrData.sizePointer+1;
        VPIKeypoint new_p;
        new_p.x = p.x;
        new_p.y = p.y;
        forwArrPoints[end_idx] = new_p; 
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
    vpiArrayUnlock(arrForwPts);
}

void pub_image(cv::Mat& img){
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<sensor_msgs::Image>("hist_image", 1);
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
  sensor_msgs::Image image;
  img_bridge.toImageMsg(image);
  pub.publish(image);

}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    // std::cerr<<"round:"<<round<<std::endl;
    // round = round + 1;
    // if (round<450) return;
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    /*
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        // ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
	//pub_image(img);
    }
    else
        img = _img;
    */
    img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    //ROS_INFO_STREAM("VPI_forw_img: " << VPI_forw_img << " " << img.rows << ", " << img.cols);
    forw_pts.clear();
    vpiArrayDestroy(arrForwPts);
    CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts));
    
    if(EQUALIZE){
      CHECK_STATUS(vpiImageSetWrappedOpenCVMat(VPI_hist_img, img));
      //cv::Mat test = img.clone();
      //CHECK_STATUS(vpiImageSetWrappedOpenCVMat(VPI_hist_img, test));
      //VPIPayload payload;
      CHECK_STATUS(vpiCreateEqualizeHist(backend, VPI_IMAGE_FORMAT_U8, &hist_payload));
      CHECK_STATUS(vpiSubmitEqualizeHist(stream, backend, hist_payload, VPI_hist_img, VPI_forw_img));
      //vpiStreamSync(stream);
      //std::swap(VPI_forw_img, VPI_hist_img);
      //pub_image(test);
    }
    else
      CHECK_STATUS(vpiImageSetWrappedOpenCVMat(VPI_forw_img, img));
    
    //add arrforwPTS finish
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U8, 0, &arrStatus));
        //CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts));
        //vector<float> err;
        // std::cerr<<"cur_pts size"<<cur_pts.size()<<std::endl;
        // Optical FLow Pyr LK replaced by VPI
        //cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        //std::cerr<<"wrap img"<<std::endl;
        CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream, backend, VPI_forw_img, pyrForwFrame));
        // std::cerr<<"submit LK"<<std::endl;
        //outputVPIKeypoints(arrCurPts);
        CHECK_STATUS(vpiSubmitOpticalFlowPyrLK(stream, backend, optflow, pyrCurFrame, pyrForwFrame, arrCurPts, arrForwPts,arrStatus, &lkParams));
        // VPI status to vector status
        // std::cerr<<"forw pts"<<std::endl;
        //outputVPIKeypoints(arrForwPts);
         CHECK_STATUS(vpiStreamSync(stream));
        VPIstatus_to_cvStatus(arrStatus,status);
        // arrCurpts to cur_pts
        VPIKeyPointArr_to_cvPointVec(arrCurPts,cur_pts);
        //arrForwpts to forw_pts
        VPIKeyPointArr_to_cvPointVec(arrForwPts, forw_pts);
        // std::cerr<<"status:"<<status.size()<<std::endl;
        // std::cerr<<"cur pts:"<<cur_pts.size()<<std::endl;
        // std::cerr<<"forw pts:"<<forw_pts.size()<<std::endl;
        // std::cerr<<"prev pts:"<<prev_pts.size()<<std::endl;
        // for (int i = 0; i < int(forw_pts.size()); i++)
        // {
        //     std::cerr<<(unsigned char)(status[i]);
        // }
        // std::cerr<<std::endl<<"start reduce"<<std::endl;
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        // for (int i = 0; i < int(forw_pts.size()); i++)
        // {
        //     std::cerr<<(unsigned char)(status[i]);
        // }

        reduceVector1(prev_pts, status,arrPrevPts);
        reduceVector1(cur_pts, status,arrCurPts);
        reduceVector1(forw_pts, status,arrForwPts);
        reduceVector(ids, status);
        reduceVector1(cur_un_pts, status);
        reduceVector(track_cnt, status);
        vpiArrayDestroy(arrStatus);
        // std::cerr<<std::endl<<"finish reduce"<<std::endl;
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {   
        // std::cout<<"pub this frame:"<<round++<<std::endl;
        rejectWithF();
        // ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        // ROS_DEBUG("set mask costs %fms", t_m.toc());

        // ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        // std::cerr<<"n_max_cnt:"<<n_max_cnt<<std::endl;
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            //replaced by VPI
            //std::cerr<<"submit harris corner"<<std::endl;
            //CHECK_STATUS(vpiArrayCreate(MAX_CNT - forw_pts.size(), VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts));
            // std::cerr<<"create array for harris"<<std::endl;
            // std::cerr<<"size:"<<MAX_CNT - forw_pts.size()<<std::endl;
            CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &VPI_n_pts));
            CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U32, 0, &scores));
            // CHECK_STATUS(vpiArrayCreate(MAX_CNT - forw_pts.size(), VPI_ARRAY_TYPE_KEYPOINT, 0, &VPI_n_pts));
            // CHECK_STATUS(vpiArrayCreate(MAX_CNT - forw_pts.size(), VPI_ARRAY_TYPE_U32, 0, &scores));
            //std::cerr<<"finish create"<<std::endl;
            // std::cerr<<"submit harris corner detector"<<std::endl;
            CHECK_STATUS(vpiSubmitHarrisCornerDetector(stream, backend, harris, VPI_forw_img, VPI_n_pts, scores, &harrisParams));
            CHECK_STATUS(vpiStreamSync(stream));
           // std::cerr<<"sort"<<std::endl;
            // SortKeypoints(VPI_n_pts, scores, MAX_KEYPOINTS);
            //std::cerr<<"finish sort"<<std::endl;
            //VPIKeyPointArr_to_cvPointVec(arrForwPts, n_pts);
            if (MAX_CNT - forw_pts.size()== 1){
                // std::cerr<<"add only one point"<<std::endl;
                SortKeypoints(VPI_n_pts,scores,1);
                VPIKeyPointArr_to_cvPointVec(VPI_n_pts, n_pts);
            }
            else{
                // std::cerr<<"start using quat tree"<<std::endl;
                n_pts = pickPtsByQuatTree(VPI_n_pts,scores,MAX_CNT - forw_pts.size());
                // std::cerr<<"finish using quad tree"<<std::endl;
            }

            //VPIKeyPointArr_to_cvPointVec(VPI_n_pts, n_pts);
            // for (int i = 0;i<n_pts.size();i++){
            //     std::cerr<<"("<<n_pts[i].x<<","<<n_pts[i].y<<")";
            // }
            // std::cerr<<std::endl;
           //cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
            vpiArrayDestroy(VPI_n_pts);
            vpiArrayDestroy(scores);
        }
        else{
            // std::cerr<<"clear n_pts"<<std::endl;
            n_pts.clear();

            //CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &VPI_n_pts));
        }
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // ROS_DEBUG("add feature begins");
        TicToc t_a;
        //std::cerr<<"add points"<<std::endl;
        addPoints();
        //std::cerr<<"finish adding points"<<std::endl;
        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    // std::cerr<<"swapping"<<std::endl;
    if(EQUALIZE)
      vpiPayloadDestroy(hist_payload);
    
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    std::swap(VPI_prev_img, VPI_cur_img);
    std::swap(VPI_cur_img, VPI_forw_img);
    std::swap(arrPrevPts, arrCurPts);
    std::swap(arrCurPts, arrForwPts);
    std::swap(pyrCurFrame, pyrForwFrame);
    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        // ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        // string filenamebase = "/home/yao/pts_test_vpi";
        // string filename = filenamebase + "/pts" + to_string(filecount)+".csv";
        // filecount++;
        // ofstream ofile;
        // std::cout<<"start ransac on cuda"<<std::endl;
        // std::cerr<<"forw_pts size"<<un_forw_pts.size()<<","<<"cur pts size"<<un_cur_pts.size()<<std::endl;
        #if USE_GPU
            RANSAC_cuda_tools::findFundamentalMat_on_cuda(un_cur_pts,un_forw_pts,0.2,0.99,status);  
        #elif
            cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);  
        #endif

        // ofile.open(filename);
        // format: cur pts, forw pts, status
        // for (int i=0;i<un_cur_pts.size();i++){
        //     ofile<<un_cur_pts[i].x<<','<<un_cur_pts[i].y<<','<<un_forw_pts[i].x<<','<<un_forw_pts[i].y<<','<<(int)status[i]<<'\n';
        // }
        // ofile.close();
        int size_a = cur_pts.size();
        // std::cerr<<"in reject---------------"<<std::endl;
        // std::cerr<<"status:"<<status.size()<<std::endl;
        // std::cerr<<"cur pts:"<<cur_pts.size()<<std::endl;
        // std::cerr<<"forw pts:"<<forw_pts.size()<<std::endl;
        // std::cerr<<"prev pts:"<<prev_pts.size()<<std::endl;
        // std::cerr<<"start reduce"<<std::endl;
        reduceVector1(prev_pts, status,arrPrevPts);//reduce vector apply also on VPI array
        reduceVector1(cur_pts, status,arrCurPts);
        reduceVector1(forw_pts, status,arrForwPts);
        reduceVector1(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // std::cerr<<"start reduce"<<std::endl;
        // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    // ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

// initialize VPI data structure
void FeatureTracker::initVPIData(const cv::Mat& img){
  /*
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  */
  //cv::Mat init_img = ptr->image.rowRange(0, ROW);
  cv::Mat init_img = img.rowRange(0, ROW);
    CHECK_STATUS(vpiStreamCreate(0, &stream));
    // CV mat wrapper
    CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(init_img, 0, &VPI_prev_img));
    CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(init_img, 0, &VPI_cur_img));
    CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(init_img, 0, &VPI_forw_img));
    CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(init_img, 0, &VPI_hist_img));
    //get format
    CHECK_STATUS(vpiImageGetFormat(VPI_prev_img, &imgFormat));
    //create pyramid
    CHECK_STATUS(vpiPyramidCreate(init_img.cols, init_img.rows, imgFormat, pyrLevel, 0.5, 0, &pyrForwFrame));
    CHECK_STATUS(vpiPyramidCreate(init_img.cols, init_img.rows, imgFormat, pyrLevel, 0.5, 0, &pyrCurFrame));
    //create array
    CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrPrevPts));
    CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrCurPts));
    CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts));
    CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U8, 0, &arrStatus));
    CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U32, 0, &scores));
    //create optical flow pyramidal LK
    CHECK_STATUS(vpiCreateOpticalFlowPyrLK(backend, init_img.cols, init_img.rows, imgFormat, pyrLevel, 0.5, &optflow));
    // set LK flow parameter
    memset(&lkParams,0,sizeof(lkParams));
    lkParams.useInitialFlow  = false;
    lkParams.termination     = VPI_TERMINATION_CRITERIA_ITERATIONS | VPI_TERMINATION_CRITERIA_EPSILON;
    lkParams.epsilonType     = VPI_LK_ERROR_L1;
    lkParams.epsilon         = 0.0f;
    lkParams.windowDimension = 15;
    lkParams.numIterations   = 6;
    // create the payload for harris corners detector
    CHECK_STATUS(vpiCreateHarrisCornerDetector(backend, init_img.cols, init_img.rows, &harris));
    harrisParams.gradientSize   = 5;
    // harrisParams.blockSize      = 5;
    harrisParams.blockSize = 7;
    harrisParams.sensitivity    = 0.01;
    harrisParams.minNMSDistance = 12;
    // CHECK_STATUS(vpiSubmitHarrisCornerDetector(stream, backend, harris, VPI_cur_img, arrCurPts, scores, &harrisParams));
    // CHECK_STATUS(vpiStreamSync(stream));
    CHECK_STATUS(vpiSubmitGaussianPyramidGenerator(stream, backend, VPI_cur_img, pyrCurFrame));
    
    CHECK_STATUS(vpiStreamSync(stream));
}
