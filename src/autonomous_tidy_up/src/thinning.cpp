#include <autonomous_tidy_up/thinning.h>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stack>
using std::stack;
#include <vector>
using std::vector;
#include <cmath>

#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

Thinning::Thinning()
{
}
/**** Thninning algorithm version 2*
*/
//Grab the patterns for the thinning algorithm
void Thinning::_grabThinPattern2(const unsigned char *image, int width, int center, specialThinPattern &p)
{
    //Nomenclatura uniforme para artigo GUO and HALL
    p.pattern[1]=image[center+3*width-3];	//Position 0=8
    p.pattern[2]=image[center+3*width  ];	//position 1
    p.pattern[3]=image[center+3*width+3];	//position 2
    p.pattern[4]=image[center        +3];	//position 3
    p.pattern[5]=image[center-3*width+3];	//position 4
    p.pattern[6]=image[center-3*width  ];	//position 5
    p.pattern[7]=image[center-3*width-3];	//position 6
    p.pattern[8]=image[center        -3];	//Position 7
    p.pattern[0]=image[center+3*width-3];	//Position 8


}
//XH calculated through NC8 -> this is G1 condition
int Thinning::_XH(const specialThinPattern &p)
{
    //modified the order of the patterns
//    int p1_ =(p.pattern[1]==0xFF) ? 0 : 1;
//    int p2_ =(p.pattern[2]==0xFF) ? 0 : 1;
//    int p3_ =(p.pattern[3]==0xFF) ? 0 : 1;
//    int p4_ =(p.pattern[4]==0xFF) ? 0 : 1;
//    int p5_ =(p.pattern[5]==0xFF) ? 0 : 1;
//    int p6_ =(p.pattern[6]==0xFF) ? 0 : 1;
//    int p7_ =(p.pattern[7]==0xFF) ? 0 : 1;
//    int p8_ =(p.pattern[8]==0xFF) ? 0 : 1;

//    int count= (p8_ - p8_ * p1_ * p2_) + (p2_- p2_ * p3_ * p4_) + (p4_- p4_ * p5_ * p6_) +  (p6_- p6_ * p7_ * p8_);

    bool p1 =p.pattern[1]==0xFF ? true : false;
    bool p2 =p.pattern[2]==0xFF ? true : false;
    bool p3 =p.pattern[3]==0xFF ? true : false;
    bool p4 =p.pattern[4]==0xFF ? true : false;
    bool p5 =p.pattern[5]==0xFF ? true : false;
    bool p6 =p.pattern[6]==0xFF ? true : false;
    bool p7 =p.pattern[7]==0xFF ? true : false;
    bool p8 =p.pattern[8]==0xFF ? true : false;
    int count= int((!p2)&&(p3||p4))+int((!p4)&&(p5||p6))+int((!p6)&&(p7||p8))+int((!p8)&&(p1||p2));

    return count;
}
//G2 condition
int Thinning::_minN1N2(const specialThinPattern &p)
{
    int N1=0, N2=0;

    bool p1 =p.pattern[1]==0xFF ? true : false;
    bool p2 =p.pattern[2]==0xFF ? true : false;
    bool p3 =p.pattern[3]==0xFF ? true : false;
    bool p4 =p.pattern[4]==0xFF ? true : false;
    bool p5 =p.pattern[5]==0xFF ? true : false;
    bool p6 =p.pattern[6]==0xFF ? true : false;
    bool p7 =p.pattern[7]==0xFF ? true : false;
    bool p8 =p.pattern[8]==0xFF ? true : false;

    //uniformizado para paper Guo
    //The N2 pattern
    N1= int(p1 || p2) + int(p3 || p4) + int(p5 || p6) + int(p7 || p8);
    //The N2 pattern
    N2=	int(p2 || p3) + int(p4 || p5) + int(p6 || p7) + int(p8 || p1);

    return min(N1, N2);
}
bool Thinning::_testG3(const specialThinPattern &p)//odd  condition
{
    bool p2 =p.pattern[2]==0xFF ? true : false;
    bool p3 =p.pattern[3]==0xFF ? true : false;
    bool p4 =p.pattern[4]==0xFF ? true : false;
    bool p5 =p.pattern[5]==0xFF ? true : false;

    //Nomenclatura GUO
    return (p2||p3||(!p5)) && (p4);
//    return (((p.pattern[2]==0xFF)||(p.pattern[3]==0xFF)||(!(p.pattern[5]==0xFF)))&&(p.pattern[4]==0xFF)) ? false : true;
}
bool Thinning::_testG3_(const specialThinPattern &p)//even condition
{
    bool p1 =p.pattern[1]==0xFF ? true : false;
    bool p6 =p.pattern[6]==0xFF ? true : false;
    bool p7 =p.pattern[7]==0xFF ? true : false;
    bool p8 =p.pattern[8]==0xFF ? true : false;
    //Nomenclatura GUO
    return (p6||p7||(!p1)) && (p8);
}

//Thinning algorithm from GUO and Hall
bool Thinning::thinningGUOandHALL(const unsigned char* image, unsigned char* output, int width, int height)
{
    if(image==NULL)
    {
        //image is null
        //std::cerr << "Received null image to thin." << std::endl;
        return false;
    }

    if(width<3 || height < 3)
        return false;

    //allocating the mark buffer
    unsigned char* MBuffer=new unsigned char[width*height*3];

    //the output image is originally equal the input image
    memcpy(output, image, width*height*3*sizeof(unsigned char));

    int turn=0;
    bool iterate=true;
    while(iterate)
    {
        //try to set the last iteration
        iterate=false;
        turn=(turn+1);

        //reset mark buffer
        for(int s=0;s<width*height*3;s+=3)
        {
            MBuffer[s]=0x00;MBuffer[s+1]=0x00;MBuffer[s+2]=0x00;
        }

        //start from white patterns
        for(int line = 1; line < height-1; ++line)
        {
            for(int col = 3; col < width*3-3; col+=3)
            {

                //Grab central position
                int center=width*3*line+col;

                //Check if it is one
                if(output[center]==0xFF)
                {
                    //declaring the pattern grabber
                    specialThinPattern p;
                    //grabbing the neighborhood
                    _grabThinPattern2(output, width, center, p);

                    //Checking conditions 1 and 2
                    bool G1= (_XH(p)==1) ?  true : false;
                    int minN1N2=_minN1N2(p);
                    bool G2= ((2<=minN1N2)&&(minN1N2<=3)) ? true : false;
                    bool G3;
                    if(turn%2==0)
                        G3 = _testG3_(p);//even
                    else
                        G3 = _testG3(p); //odd

                    //If conditions are satisfied erase pixel
                    if((G1==true) && (G2==true) && (G3==false))
                    {
                        MBuffer[center  ]=M;
                        MBuffer[center+1]=M;
                        MBuffer[center+2]=M;

                        iterate=true;
                    }

                }
            }
        }
        //Now delete pixels from pass P1 or P2, depending on what turn it is
        for(int line = 0; line < height; ++line)
            for(int col = 0; col < width*3; col+=3)
            {
                if(MBuffer[width*3*line+col]==M)
                {
                    output[width*3*line+col]=0x00;
                    output[width*3*line+col+1]=0x00;
                    output[width*3*line+col+2]=0x00;
                }
            }
    }

    _setBordersAsZero(output, width, height, 3);

    //erasing the temporary buffer
    if(MBuffer!=NULL)
        delete [] MBuffer;
    MBuffer=NULL;
    return true;
}

void Thinning::_setBordersAsZero(unsigned char *image, const size_t& width, const size_t& height, const size_t& channels)
{
    //Setting lower border
    int numPixelsLine=width*channels;
    for(int i=0;i<numPixelsLine;i++)
    {
        for(unsigned int c=0;c<channels;c++)
            image[i+c]=0x00;
    }
    //discovering upper border
    int last=width*height*channels-channels;
    int fol=last - numPixelsLine;
    for (int i=fol;i<last;i++)
    {
        for(unsigned int c=0;c<channels;c++)
            image[i+c]=0x00;
    }
    //Setting left border
    for(int i=0;i<last;i+=numPixelsLine)
    {
        for(unsigned int c=0;c<channels;c++)
            image[i+c]=0x00;
    }
    //Setting right border
    for(int i=numPixelsLine-3;i<last;i+=numPixelsLine)
    {
        for(unsigned int c=0;c<channels;c++)
            image[i+c]=0x00;
    }
}

bool Thinning::spurRemoval(int iterations, unsigned char* image, int width, int height)
{
    if(image==NULL)
    {
        //image is null
        //std::cerr << "Received null image to thin." << std::endl;
        return false;
    }

    if(width<3 || height < 3)
        return false;

    int offset[][8]={{-3,  3},
                     { 0,  3},
                     { 3,  3},
                     { 3,  0},
                     { 3, -3},
                     { 0, -3},
                     {-3, -3},
                     {-3,  0}};

    //list of possible spur segments
    vector< vector<int> > possibleSpurs;

    stack<int> st;
    int intercection[8]={-1, -1, -1, -1, -1, -1, -1, -1};
    //start from white patterns
    for(int line = 1; line < height-1; ++line)
    {
        for(int col = 3; col < width*3-3; col+=3)
        {
            //temporary storage of the tip of the spur
            vector<int> ending;

            //Grab central position
            int center=width*3*line+col;
            //Check if the current point is thinning point
            if(image[center]==0xFF){
                int counting=0;
                for(int i=0;i<8;++i)
                    if(image[center+offset[i][1]*width+offset[i][0]]==0xFF)
                    {
                        counting++;
                    }

                //Add thinning edge point to the list
                if(counting==1){
                    st.push(center);
                }

                // isolated thinning point removal
                if(counting==0){
                    image[center  ]=0x00;
                    image[center+1]=0x00;
                    image[center+2]=0x00;
                }
            }
        }
    }

    // Remove spurious edge lines up to a predefined limit
    while(!st.empty()){
        int center = st.top();
        st.pop();

//        image[center  ]=0x80;
//        image[center+1]=0x80;        
//        image[center+2]=0x80;

        int iter=0;
        while(iter<iterations){
            int neighbor;
            int counting=0;
            for(int n=0;n<8;++n){
                if(image[center+offset[n][1]*width-offset[n][0]]==0xFF){
                    neighbor=center+offset[n][1]*width-offset[n][0];
                    counting++;
                }
            }
            // Remove the thinning point only if it is still a edge point
            // (I guess this removes thinning discontinuities)
            if(counting==1){
                if(image[center]==0xFF){
                    image[center  ]=0x00;
                    image[center+1]=0x00;
                    image[center+2]=0x00;
                    center=neighbor;
                }
            }else{
                break;
            }
            iter++;
        } 
    }

   return true; 
}

bool Thinning::spurRemovalSmart(int iterations, unsigned char* image, int width, int height)
{
    //Checking if the map is too small
    if(image==NULL)
    {
        //image is null
        //std::cerr << "Received null image to thin." << std::endl;
        return false;
    }

    if(width<3 || height < 3)
        return false;


    //displacement used across neighbors (it is three because it is RGB, for historical reasons)
    int offset[][9]={{-3,  3},
                     { 0,  3},
                     { 3,  3},
                     { 3,  0},
                     { 3, -3},
                     { 0, -3},
                     {-3, -3},
                     {-3,  0}};

    //list of possible spur segments
    vector< vector<int> > possibleSpurs;

    //start from white patterns
    for(int line = 1; line < height-1; ++line)
    {
        for(int col = 3; col < width*3-3; col+=3)
        {
            //temporary storage of the tip of the spur
            vector<int> ending;

            //Grab central position
            int center=width*3*line+col;
            //Check if the current point is thinning point
            if(image[center]==0xFF){
                int counting=0;
                for(int i=0;i<8;++i)
                    if(image[center+offset[i][1]*width+offset[i][0]]==0xFF)
                    {
                        counting++;
                    }

                //Add thinning edge point to the list
                if(counting==1){
                    ending.push_back(center);
                    possibleSpurs.push_back(ending);
                }

                // isolated thinning point removal
                if(counting==0){
                    image[center  ]=0x00;
                    image[center+1]=0x00;
                    image[center+2]=0x00;
                }
            }
        }
    }

    // For each ending I have found, determine the size of the spurs
    for(unsigned int i=0;i<possibleSpurs.size();++i)
    {
        //get the endind as a starting point
        int center = possibleSpurs[i][0];

        // used to count neighbors
        int counting = 0;

        // Check if it still has only o neighbor
        for(int j=0;j<8;++j)
            if(image[center+offset[j][1]*width+offset[j][0]]==0xFF)
            {
                counting++;
            }


        //Add thinning edge point to the list
        int segmentSize = 1; // every segment starts with the lonely starting point
        int currPos = 0; // starting position in the vector (starts with the first, i.e., 0)
        int neighbor = 0; //neighbor
        int previousN = -1;
        if(counting==1){
            // ok =, I am an ending, what is the size of my segment and what are my neighbors?
            while(true)
            {

                // reset neighbor counting
                counting = 0;
                int store;
                center = possibleSpurs[i][currPos];
                // Check the number of neighbors
                for(int n=0;n<8;++n)
                {
                    neighbor=center+offset[n][1]*width+offset[n][0];
                    if(image[neighbor]==0xFF)
                    {
                        //check if I can sum additional neighbors (cannot be the first again, cannot be the previous one)
                        //if(neighbor!=center)
                                counting++;
                                if(neighbor!=previousN)
                                        store = neighbor;

                    }
                }
                ////////////////////////////////////////////////
                //    check if I can push_back the neighbor   //
                ////////////////////////////////////////////////
                // I have no additional neighbors besides me
                if(counting==1 && currPos!=0)
                {
//                    std::cout << "Terminei a linha" << std::endl;
                    break;
                }

                //ok I have only one NEW neighbor, moving on
                if(counting==2 ||
                        (counting==1 && currPos==0))
                {
//                    std::cout << "Entrei" << std::endl;
                    possibleSpurs[i].push_back(store);
                    previousN = center;
//                    std::cout << "Sai" << std::endl;

                    ++currPos;
                    ++segmentSize;
                }
                // whatever the decision about this pixel, it is an end point (included or not in the spur);
                if(counting >=3)
                {
//                    std::cout << "Terminei a linha" << std::endl;
                    // I am in a possibly complicated situation...
                    // I might be destroying a connection between two non-adjacent neighbors
                    break;
                }
            }
        }
    }


//    // calculate mean and standard deviation
//    double spurSizeMean = 0.0;
//    double spurSizeVar = 0.0;
//    for(int i=0;i<possibleSpurs.size();++i)
//        spurSizeMean += double(possibleSpurs[i].size());
//    spurSizeMean/=double(possibleSpurs.size());
//    //calculate the variance
//    for(int i=0;i<possibleSpurs.size();++i)
//        spurSizeVar+=pow(possibleSpurs[i].size()-spurSizeMean, 2.0);
//    spurSizeVar/=double(max(possibleSpurs.size()-1, 1)); // avoid division by zero
//    // autosetting the spur size cuttoff threshold
    double cutoff = 0.0; //sqrt(spurSizeVar)*2.0;
//    cout << cutoff << endl;
    cutoff=iterations;
    int center=0;

    // starting to trim the spurious lines
    for(unsigned int i=0;i<possibleSpurs.size();++i)
    {
        // Check if the segment is big enough
        if(possibleSpurs[i].size()<cutoff)
        {
            //jumping the last point
            for(unsigned int s=0; s<possibleSpurs[i].size();++s)
            {
                center = possibleSpurs[i][s];
                image[center  ]=0x00;
                image[center+1]=0x00;
                image[center+2]=0x00;
            }
        }
    }

    //displacement used across neighbors considering horizontal and vertical neighbours
    int offsetHeV[][4] ={{-3,  3},
                         { 3,  3},
                         { 3, -3},
                         {-3, -3}};

   return true;
}
