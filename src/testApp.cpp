#include "testApp.h"

const int Nx=640;
const int Ny=480;

cv::Mat MatInputGray;

 cv::Mat grad_x, grad_y;
 cv::Mat EdgesTemplate;
const int DeltaPhi = 36;
int TableSize = 360/DeltaPhi;
vector< vector<ofVec2f> > RTable(TableSize);
ofVec2f Refpoint; // reference point
vector<vector<cv::Point> > contours; // template points


// for scale change:
const int ScaleArraySize = 4;
float minSize = 0.5;
float maxSize = 2.0;
float ScaleTable[ScaleArraySize];

// for orientation change:
const int AngleArraySize = 8;
float minAngle = 0;
float maxAngle = 2.0*CV_PI;
float AngleTable[AngleArraySize];


// The acummulation Matrix
//vector<vector <vector<int> > > A(Ny,vector<vector<int> >(Nx,vector<int>(ScaleArraySize)));

// Four dimentional Vector
vector<vector <vector<vector<int> > > > A(Ny,vector<vector<vector<int> > >(Nx,vector<vector<int> >(ScaleArraySize,vector<int>(AngleArraySize))));

cv::Mat EdgesInput;
ofVec4f TheMaxPlace;
float TheMax;


bool ForeGroundInversion = false;
int EdgeParamOffset =0;
//--------------------------------------------------------------
void testApp::setup(){
    
  
    A.reserve(Ny);
    for(int k =0; k < Ny;k++){
        for(int q=0; q<Nx;q++){
            for (int l=0; l < ScaleArraySize; l++) {
                for (int b=0; b < AngleArraySize; b++) {
                   A[k][q][l][b]=0;
                }
            }
        }
    }
    
    // Filling the scale vector
    
    for (int k = 0; k < ScaleArraySize; k++) {
        ScaleTable[k] = minSize + (maxSize -minSize)/(float)(ScaleArraySize-1)*k;

    }

    // filling the angle vector
    for (int k = 0; k < AngleArraySize; k++) {
        AngleTable[k] = minAngle + (maxAngle -minAngle)/(float)AngleArraySize*k;
     
    }
    
    vidGrabber.setVerbose(true);
    vidGrabber.initGrabber(Nx,Ny);
    colorImg.allocate(Nx,Ny);
	grayImage.allocate(Nx,Ny);
    PatternRead.loadImage("two.png");
    PatternColor.allocate(PatternRead.width, PatternRead.height);
    PatternColor.setFromPixels(PatternRead.getPixels(),PatternRead.width,PatternRead.height);
    PatternGray.allocate(PatternRead.width, PatternRead.height);
    PatternGray = PatternColor;
    TargetCV = cvCreateImage(cvSize(PatternRead.width, PatternRead.height), IPL_DEPTH_8U, 1);
    TargetCV = PatternGray.getCvImage();
    //extracting the gradient information of the template.
    cv::Mat Graytemplate(TargetCV);
    
    
    
    cv::Sobel( Graytemplate, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    
    cv::Sobel( Graytemplate, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

   
    
    // vector of vector of contours
    
    vector<cv::Vec4i> hierarchy;
    
    /// Detect edges using canny
    // edges of the template
    cv::Canny(Graytemplate, EdgesTemplate, 50, 10);
    //Canny( src_gray, canny_output, thresh, thresh*2, 3 );

    /// Find contours
    cv::findContours( EdgesTemplate, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
    
    /// Get the moments
    vector<cv::Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    { mu[i] = moments( contours[i], false ); }
    
    ///  Get the mass centers:
//    vector<cv::Point2f> mc( contours.size() );
//    for( int i = 0; i < contours.size(); i++ )
//    { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
    
    Refpoint.x = mu[0].m10/mu[0].m00;
    Refpoint.y = mu[0].m01/mu[0].m00;

    // looping trought the contour and filling the RTable;
    

    for( int k = 0; k < contours[0].size(); k++ )
    {
    
        // checking the gradient in the point position
        ofVec2f Pos;
        
        Pos.x = contours[0][k].x;
        Pos.y = contours[0][k].y;
        
        float TheGrad = atan2f((float)grad_y.at<signed short>(Pos.y,Pos.x), (float)grad_x.at<signed short>(Pos.y,Pos.x));
        // result will be (-pi,pi]
        
        TheGrad = (TheGrad>0)?TheGrad:(TheGrad + 2*CV_PI);
        // now thegrad e [0 2pi]
        
        // finding the apropiate bin
        
        int TheBin = TheGrad*180/CV_PI/DeltaPhi;
        if (TheBin == TableSize){TheBin=0;}
        

        // storing the vector poin on that bin
        RTable[TheBin].push_back(Refpoint-Pos);
        
    }

Fuente.loadFont("helvetica.ttf", 32);
}

//--------------------------------------------------------------
void testApp::update(){
    bool bNewFrame = false;
    
    // cleanning the acumulator
    
    for(int k =0; k < Ny;k++){
        for(int q=0; q<Nx;q++){
            for (int l=0; l < ScaleArraySize; l++) {
                for (int b=0; b < AngleArraySize; b++) {
                    A[k][q][l][b]=0;
                }
            }
        }
    }
    
    vidGrabber.update();
    bNewFrame = vidGrabber.isFrameNew();

    cv::Mat InputGradX;
    cv::Mat InputGradY;
    
      EdgesInput.create(Ny, Nx, CV_8U);
	if (bNewFrame){
        
        colorImg.setFromPixels(vidGrabber.getPixels(), Nx,Ny);
        grayImage = colorImg;
        MatInputGray = cv::Mat(grayImage.getCvImage());
        // gradients
        cv::Sobel( MatInputGray, InputGradX, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
        
        cv::Sobel( MatInputGray, InputGradY, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
        
        // Image Edges
        cv::Canny(MatInputGray, EdgesInput, 160 +EdgeParamOffset, 100);
        
 
        // searching on the edge pixels
        for (int x =0; x<Nx; x++) {
            for (int y =0; y < Ny; y++) {
                // checking if it is an edge Pixel or not
                if (EdgesInput.at<uchar>(y,x)!=0) {
                    
                    // check for the gradient at that point
                    
                    float TheGrad = atan2f((float)InputGradY.at<signed short>(y,x), (float)InputGradX.at<signed short>(y,x));
                        // result will be (-pi,pi]
                    
                       // 2 searchs for inverted countours
                    int FI = (ForeGroundInversion==true)?2:1;
                    for (int FIcon = 0; FIcon <FI; FIcon++) {
                    
                        
                        // loop for the rotation
                         for (int b=0; b < AngleArraySize; b++) {
                             float NGrad = TheGrad - AngleTable[b] -FIcon*CV_PI;
                             
                             while (NGrad<0) {
                                 NGrad +=  2*CV_PI;
                             }
                             // now thegrad e [0 2pi]
                    
                            // finding the apropiate bin
                            int TheBin = NGrad*180/CV_PI/DeltaPhi;
                            if (TheBin == TableSize){TheBin=0;}

                               // Getting the coordinates of the vectors stored
                    
                               for (int k=0; k< RTable[TheBin].size(); k++) {
                        
                               // Testing for different scales
                               for (int s = 0; s < ScaleArraySize; s++) {
                                   float ScX,ScY;
                                   ScX = ScaleTable[s]*RTable[TheBin][k].x;
                                   ScY = ScaleTable[s]*RTable[TheBin][k].y;
                                   int Xacc = x + cosf(AngleTable[b])*ScX -sinf(AngleTable[b])*ScY;
                                   int Yacc = y + sinf(AngleTable[b])*ScX + cosf(AngleTable[b])*ScY;
                        
                        
                                    // Check if it is a valid coordinate and increment the acummulator

                                    if ((Xacc>=0) && (Xacc < Nx) && (Yacc >=0)&&(Yacc <Ny)) {
                                        A[Yacc][Xacc][s][b]+=1;
                                     }
                                } // closing the scale for loop
                                   
                             }// closing RTable loop
                         } // closing Angle loop
                     } // closing the foreground inversion loop
                   } // closing the "it is an edge" loop
            
            } // closing y loop
        } // closing x loop
      
       
        // Looking for the Maximun
        // searching on the edge pixels

        TheMax = 0.0;
        for (int x =0; x<Nx; x++) {
            for (int y =0; y < Ny; y++) {
                for (int s=0; s < ScaleArraySize; s++) {
                    for (int b=0; b < AngleArraySize; b++) { 
                       if (A[y][x][s][b]>TheMax) {
                         TheMax = A[y][x][s][b];
                         TheMaxPlace.x = x;
                         TheMaxPlace.y = y;
                         TheMaxPlace.z = s;
                         TheMaxPlace.w = b;
                       }
                    }
                }
            
            }
        }
        
        
    }
    
}




//--------------------------------------------------------------
void testApp::draw(){
	ofSetHexColor(0xffffff);
    PatternGray.draw(0, 0);
    ofxCvGrayscaleImage auxDraw1;
    cv::Mat rescale1;
    double minVal, maxVal;
    ofColor TheColor;
    TheColor.setHsb(TheMax, 128, 128);
   //    minMaxLoc(Testcumulator, &minVal, &maxVal,NULL,&maxPlace); //find minimum and maximum intensities
//    rescale1.create(Testcumulator.rows, Testcumulator.cols, CV_8U);
//    Testcumulator.convertTo(rescale1, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
//    //ofSetMinMagFilters();
//    auxDraw1.allocate(rescale1.cols, rescale1.rows);
//    auxDraw1 = rescale1.data;
//    auxDraw1.draw(PatternGray.width,0);
    
    
    ofxCvGrayscaleImage auxDraw2;
    auxDraw2.allocate(EdgesInput.cols, EdgesInput.rows);
    auxDraw2 = EdgesInput.data;
    auxDraw2.draw(PatternGray.width,0);
    grayImage.draw(PatternGray.width+auxDraw2.width, 0);
    
    ofSetColor(TheColor);
    ofCircle(TheMaxPlace.x+PatternGray.width , TheMaxPlace.y, 2);
    ofCircle(TheMaxPlace.x+PatternGray.width +auxDraw2.width , TheMaxPlace.y, 2);
    glBegin(GL_LINE_STRIP);
    
    for( int k = 0; k < contours[0].size(); k++ ){
        float Xk =contours[0][k].x;
        float Yk = contours[0][k].y;
        float Cx = Refpoint.x;
        float Cy = Refpoint.y;
        float S = ScaleTable[(int)TheMaxPlace.z];
        float OffX = TheMaxPlace.x;
        float OffY = TheMaxPlace.y;
        float sinB = sinf(AngleTable[(int)TheMaxPlace.w]);
        float cosB = cosf(AngleTable[(int)TheMaxPlace.w]);
        float Xcord;
        float Ycord;
        Xcord = S*cosB*(Xk-Cx)-S*sinB*(Yk-Cy)+OffX;
        Ycord = S*sinB*(Xk-Cx)+S*cosB*(Yk-Cy)+OffY;
        glVertex2f(Xcord + PatternGray.width,Ycord);
    }
    
    glEnd();
    
    glBegin(GL_LINE_STRIP);
    
    for( int k = 0; k < contours[0].size(); k++ ){
        float Xk =contours[0][k].x;
        float Yk = contours[0][k].y;
        float Cx = Refpoint.x;
        float Cy = Refpoint.y;
        float S = ScaleTable[(int)TheMaxPlace.z];
        float OffX = TheMaxPlace.x;
        float OffY = TheMaxPlace.y;
        float sinB = sinf(AngleTable[(int)TheMaxPlace.w]);
        float cosB = cosf(AngleTable[(int)TheMaxPlace.w]);
        float Xcord;
        float Ycord;
        Xcord = S*cosB*(Xk-Cx)-S*sinB*(Yk-Cy)+OffX;
        Ycord = S*sinB*(Xk-Cx)+S*cosB*(Yk-Cy)+OffY;
        glVertex2f(Xcord + PatternGray.width+auxDraw2.width,Ycord);
    }
    
    glEnd();
 
    
    
}












//--------------------------------------------------------------
void testApp::keyPressed(int key){
    
    switch (key) {
        case 'f':
            ForeGroundInversion=!ForeGroundInversion;

            break;
            
        case 'r':
      
            break;
        case OF_KEY_RIGHT:
       
            break;
        case OF_KEY_LEFT:
       
            break;
        case OF_KEY_UP:
            EdgeParamOffset++;
            cout<<EdgeParamOffset<<endl;
            break;
        case OF_KEY_DOWN:
            EdgeParamOffset--;
            if (EdgeParamOffset<=110)
            {EdgeParamOffset =110;}
            cout<<EdgeParamOffset<<endl;
            break;
        case 'a':
 
            break;
            
        case 's':
       
            break;
        case 'q':
          
            break;
        default:
            break;
    }
    
    
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y){
    
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 
    
}