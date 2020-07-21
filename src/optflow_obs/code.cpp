///****************** 4- 避障决策与控制 *********************///
/// 4-0 通用
// 航向角
const float nav_yaws[] = {-35,-25,-15,-5,0,5,15,25,35};
const vector<float> nav_sgn = {-1,-1,-1,-1,1,1,1,1};

// 划分竖直条纹
vector<int > linspaceMat(int width,int stroop){
    vector<int> res(stroop+1);
    res[0] = 0;
    int stride = width/stroop;
    for(int i=1;i<stroop+1;++i){
        res[i] = stride+res[i-1];
    }

    return res;
}

// 绘制竖条
void plotVbar(Mat &img,int amuzi,int stroop=8){
    int width = img.cols;
    int height = img.rows;
    vector<int> stroops = linspaceMat(width,stroop);
    int j=0;
    for(auto ele:stroops){
        if(j==amuzi || j==amuzi+1){
            cv::line(img,Point2d(ele,0),Point2d(ele,height),Scalar(0,255,0),4,LINE_8);
        }else{
            cv::line(img,Point2d(ele,0),Point2d(ele,height),Scalar(120,0,0),2,LINE_8);
        }
        j++;
    }
    putText(img,"ENTRY",Point(cvRound((2*amuzi+1)*width/(2*stroop)), cvRound(height/2)),
            FONT_HERSHEY_PLAIN,1.0,Scalar(255,255,0),2,LINE_AA,false);
}

// 简易寻找地平线
int findHorizontalLine(Mat ttc){
    int res=0;
    for(int i=0;i<ttc.rows;++i){
        bool flag = true;
        for(int j=0;j<ttc.cols;++j){
            if(ttc.at<uchar >(i,j)>100){
                flag=false;
                break;
            }
        }
        if(flag){
            res=i;
            break;
        }
    }
    return res;
}

// 求矢量的平均值
float getMean(vector<float> vec){
    if(vec.size()==0) return 0.0;
    float res = 0.0;
    for(auto ele:vec){
        res += ele/vec.size();
    }
    return res;
}

// 把矢量归一化
void normVec(vector<float> &input){
    float minV = FLT_MAX;
    float maxV = -FLT_MIN;
    for(auto ele:input){
        if(ele<minV) minV = ele;
        if(ele>maxV) maxV = ele;
    }
    cout<<"minV:"<<minV<<" maxV:"<<maxV<<endl;
    sort(input.begin(),input.end());
    for(int i=0;i<input.size();++i){
        input[i] = (input[i]-minV)/(maxV-minV);
    }
}

/// 4-1 平衡法
/// 计算每个竖条的平均光流幅值
vector<float> getVbarMag(Mat mag,int stroop=8){
    vector<float> res(stroop,0);

    int height = mag.rows;
    int width = mag.cols;
    vector<int> lin = linspaceMat(width,stroop);
    float stroop_area = width*height/stroop;
    for(int i=0;i<stroop;++i){
        int low = lin[i];
        int high = lin[i+1];
        for(int col =low; col<high; ++col){
            for(int row=0; row<height; ++row){
                res[i] += mag.at<float >(row,col)/stroop_area;
            }
        }
    }
    return res;
}

// 计算航向角
float getNavYaw(int barInd){
    barInd -=4;
    return nav_yaws[barInd]*M_PI/180;
}

// 基于光流平均幅值进行航向角决策
float getBalancedYawBaseOnMagnitude(vector<float> bar_mags,int stroops,float threshold = 1.10){
    float yaw;
    if(stroops==2){
        if(bar_mags[0]>threshold*bar_mags[1]){
            yaw = -M_PI/8; //左侧幅值大，更近；往右偏，顺时针旋转
        }else if(bar_mags[1]>threshold*bar_mags[0]){
            yaw = M_PI/8; //右侧幅值大，更近；往左偏，逆时针旋转
        }else{
            yaw = 0.0; //近似相等，不改变航向
        }
    }else{
        int maxBarMagInd = 0;
        float maxBarMag = 0.0;
        for (int j = 0; j < bar_mags.size(); ++j) {
            if(bar_mags[j]>maxBarMag){
                maxBarMag = bar_mags[j];
                maxBarMagInd = j;
            }
        }
        yaw = getNavYaw(maxBarMagInd);
    }
    return yaw;
}

// 基于光流幅值左右平衡避障，根据差值计算航向角
// method=2表示根据差值计算航向角，=1表示固定值
float getLRBalancedYawBaseOnMagnitude(vector<float> bar_mags,bool DIFF_METHOD = true,float maxYaw = 35.0,float threshold = 1.10){
    float yaw;
    if(bar_mags[0]>threshold*bar_mags[1]){
        yaw = DIFF_METHOD? -maxYaw*(bar_mags[0]-bar_mags[1])/(bar_mags[0]+bar_mags[1]): -M_PI/8; //左侧幅值大，更近；往右偏，顺时针旋转
    }else if(bar_mags[1]>threshold*bar_mags[0]){
        yaw = DIFF_METHOD? maxYaw*(bar_mags[0]-bar_mags[1])/(bar_mags[0]+bar_mags[1]): M_PI/8; //右侧幅值大，更近；往左偏，逆时针旋转
    }else{
        yaw = 0.0; //近似相等，不改变航向
    }

    return yaw;
}

// 基于幅值平衡策略获取偏航角
float getNavYawByBS(Mat mag,int stroops){
    vector<float> bar_mags = getVbarMag(mag,stroops);
    return getBalancedYawBaseOnMagnitude(bar_mags,stroops);
}


/// 4-2 多竖条航向决策
Point2f getAmuziVel(int amuzi,float mag = 2.0){
    Point2f vel;
    vel.y = mag;
    amuzi -= 4;
    vel.x = vel.y*tan(nav_yaws[amuzi]*M_PI/180);

    return vel;
}

// 计算偏航角
float getSteeringYaw(int amuzi){
    amuzi -= 4;
    return nav_yaws[amuzi]*M_PI/180;
}

/// 计算每个竖条的平均TTC
vector<float> getVbarTTC(Mat ttcResult,int stroop=8){
    vector<float> res(stroop,0);

    int height = ttcResult.rows;
    int width = ttcResult.cols;
    vector<int> lin = linspaceMat(width,stroop);
    float stroop_area = width*height/stroop;
    for(int i=0;i<stroop;++i){
        int low = lin[i];
        int high = lin[i+1];
        for(int col =low; col<high; ++col){
            for(int row=0; row<height; ++row){
                res[i] += ttcResult.at<uchar >(row,col)/stroop_area;
            }
        }
    }

    return res;
}
