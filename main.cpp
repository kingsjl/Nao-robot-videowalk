

#include <iostream>
#include <alerror/alerror.h>
#include <alcommon/albroker.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/almotionproxy.h>
#include<opencv2/opencv.hpp>
#include<windows.h>
#define max_corners 50
using namespace AL;
using namespace cv;

boost::mutex mtx_;



boost::shared_ptr<ALVideoDeviceProxy> videoProxy;
boost::shared_ptr<ALMemoryProxy> memoryProxy;
boost::shared_ptr<ALMotionProxy> motionProxy;


int cornersCount=max_corners;
int i=0;
bool isruning=true;
int boundary[6]={-1};
int model=0;
float distance=0;
float trunangle=0;

/****************************************************************************/
//¶ÔÉÏÃæÉãÏñÍ·»ñÈ¡Í¼ÏñºÍ´¦ÀíÍ¼ÏñËùÐèµÄ±äÁ¿½øÐÐ¶¨Òå
/****************************************************************************/
std::string kTopCamera_name;
ALValue kTopCamera_curImage;
cv::Mat kTopCamera_currentImage = cv::Mat::zeros(240, 320, CV_8UC1);     //¶¨ÒåÒ»¸ö´óÐ¡Îª240*320µÄ16Î»µ¥Í¨µÀ£¬ÃûÎªkTopCamera_currentImageµÄÍ¼Ïñ¾ØÕó
Mat kTopCamera_pBinaryImage = cv::Mat::zeros(240, 320, CV_8UC1);
Mat kTopCamera_result(kTopCamera_pBinaryImage.size(),CV_8U,Scalar(0));
   
vector<vector<Point>> kTopCamera_contours;
vector<Point2f> kTopCamera_corners;
vector<Point2f> kTopCamera_corners1;
vector<Point2f> kTopCamera_cornersx;

/****************************************************************************/
//¶ÔÏÂÃæÉãÏñÍ·»ñÈ¡Í¼ÏñºÍ´¦ÀíÍ¼ÏñËùÐèµÄ±äÁ¿½øÐÐ¶¨Òå
/****************************************************************************/
std::string kBottomCamera_name;
ALValue kBottomCamera_curImage;
cv::Mat kBottomCamera_currentImage = cv::Mat::zeros(240, 320, CV_8UC1);
Mat kBottomCamera_pBinaryImage = cv::Mat::zeros(240, 320, CV_8UC1);
Mat kBottomCamera_result(kBottomCamera_pBinaryImage.size(),CV_8U,Scalar(0));
   
vector<vector<Point>> kBottomCamera_contours;
vector<Point2f> kBottomCamera_corners;
vector<Point2f> kBottomCamera_corners1;
vector<Point2f> kBottomCamera_cornersx;


/****************************************************************************/
//¿ªÊ¼Ò»¸öÏß³Ì£¬ÓÃÀ´¶Ô»ñÈ¡ÉÏÃæÉãÏñÍ·Í¼ÏñºÍ´¦Àí·ÖÎöÍ¼Ïñ£¬²¢´«³öÒ»Ð©²ÎÊýÃüÁîÐÐ×ß
/****************************************************************************/
void kTop_Camera()
{
   mtx_.lock();
   while(true)
   {
	  
   kTopCamera_currentImage.data =0;                                                //Çå¿ÕkTopCamera_currentImage¾ØÕóµÄÊý¾Ý
   kTopCamera_curImage = videoProxy->getImageRemote(kTopCamera_name);                  //ÌáÈ¡ÉãÏñÍ·µÄËù¼ì²âµÄÖµ
   kTopCamera_currentImage.data = (uchar*) kTopCamera_curImage[6].GetBinary();         //¸³¸økTopCamera_currentImage¾ØÕó
   
   threshold(kTopCamera_currentImage,kTopCamera_pBinaryImage,130,254,CV_THRESH_BINARY); //¶ÔÍ¼Ïñ½øÐÐ¶þÖµ»¯
   
   findContours(kTopCamera_pBinaryImage, kTopCamera_contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  //¼ì²âÂÖÀª£¬²¢¸³¸økTopCamera_contoursÕâ¸ö¶¯Ì¬Êý×é
   drawContours(kTopCamera_result,kTopCamera_contours,-1,Scalar(255,0,0),1,8);                       //ÃèÊöÂÖÀªÍ¼Ïñ£¬²¢¸³¸økTopCamera_result¾ØÕó
   
   goodFeaturesToTrack(kTopCamera_result,kTopCamera_corners,cornersCount,0.05,30,Mat(),3,0,0.04);      //¼ì²âÂÖÀª½Çµã
   for (i=0;i<kTopCamera_corners.size();i++)
       {
		 circle(kTopCamera_result,cvPoint((int)(kTopCamera_corners[i].x),(int)(kTopCamera_corners[i].y)),5,CV_RGB(10,10,255),2,CV_AA,0);   //ÔÚÍ¼ÏñÖÐÈ¦×¡Ëù¼ì²âµ½µÄ½Çµã
		}
   
     
  
	for (i=0;i<kTopCamera_corners.size();i++)
       {
		 kTopCamera_cornersx.push_back(kTopCamera_corners[i]);       //kTopCamera_contoursÕâ¸ö¶¯Ì¬Êý×éµÄÊý¾Ý¸´ÖÆµ½kTopCamera_cornersx¶¯Ì¬Êý×é
		} 
	 
	for (i=0;i<kTopCamera_corners.size();i++)                           /*******************************************************************/
	 {
	                                                                  //½øÐÐ½ÇµãY×ø±êÃ°ÅÝÅÅÐò£¬²¢½«Êý¾Ý¸³¸økTopCamera_corners
       for(int j=kTopCamera_corners.size()-1;j>=i;j--)
	   {
		   if(j>0)
		   {
		    if((int)kTopCamera_corners[j].y<(int)kTopCamera_corners[j-1].y)
		     {
			   kTopCamera_corners1.push_back(kTopCamera_corners[j-1]);
			   kTopCamera_corners[j-1]=kTopCamera_corners[j];
			   kTopCamera_corners[j]=kTopCamera_corners1[0];
			   
			   kTopCamera_corners1.pop_back();
			   
		     }
			
		    if((int)kTopCamera_cornersx[j].x<(int)kTopCamera_cornersx[j-1].x)      //½øÐÐ½ÇµãX×ø±êÃ°ÅÝÅÅÐò£¬²¢½«Êý¾Ý¸³¸økTopCamera_cornersx
		     {
			   kTopCamera_corners1.push_back(kTopCamera_cornersx[j-1]);
			   kTopCamera_cornersx[j-1]=kTopCamera_cornersx[j];
			   kTopCamera_cornersx[j]=kTopCamera_corners1[0];
			   
			   kTopCamera_corners1.pop_back();
			   
		     }
		   }
		 else 
			 break;
	   }
	 }                                                     /*********************************************************************************/
     std::cout<<std::endl;
	 
	 int b=0;
	 if(boundary[0]!=-1)
	 {
	 for (i=0;i<kTopCamera_cornersx.size();i++)                                    /*****************************************************/
       {
		   if(kTopCamera_cornersx[i].x>boundary[b]+10&&kTopCamera_cornersx[i].x<boundary[b+1]-10)     //ÓÃÉÏÃæÉãÏñÍ·À´¼ì²â ÏÂÃæÉãÏñÍ·ËùËÑË÷µ½µÄ¿ÕÎ»ÇøÓò£¨¿ÉÒÔÓÐ¼¸¸ö£© ÊÇ·ñ¼ì²âµ½½Çµã
		   {
			   b=b+2;                                                                         //Èô¼ì²âµ½£¬ÔòËµÃ÷ÏÂÃæÉãÏñÍ·ËùËÑË÷µ½µÄ¿ÕÎ»ÇøÓòµÄÇ°ÃæÓÐÕÏ°­£¬ÔÙ¼ì²âÏÂÒ»¸ö¿ÕÎ»ÇøÓò£¬Ö±µ½ËùÓÐ¿ÕÎ»¼ì²âÍê

			   continue;                                                                       //Ò»µ©¼ì²âÃ»ÓÐ½Çµã£¬ÔòËµÃ÷ÏÂÃæÉãÏñÍ·ËùËÑË÷µ½µÄ¿ÕÎ»ÇøÓòµÄÇ°ÃæÃ»ÓÐÕÏ°­£¬ÔòÌø³öÑ­»·   
		   }
		}
	 if(b==0||b==2)                                                                       //Èç¹ûÕÒµ½ Ã»ÕÏ°­µÄ¿ÕÎ»ÇøÓò£¬ÔòËã³ö¿ÕÎ»ÇøÓòµÄÖÐ¼ä£¬²¢ÃüÁî»úÆ÷ÈËÒÆµ½ÖÐ¼ä
	 {
		if((boundary[b]+boundary[b+1])/2<=160)                                                    //Èç¹ûÃ»ÕÒµ½£¬ÔòÃüÁî»úÆ÷ÈË×ªÍä
		{
	    model=2;
		distance=(boundary[b]+boundary[b+1])/1320;                                               //model=1ÊÇ×ªÍä£¬model=2ÊÇ×óÒÆ£¬model=3ÊÇÓÒÒÆ
		}
		else
		{model=3;
		distance=(boundary[b]+boundary[b+1])/1320;
		}
	 }
	 else
	 { 
		if((boundary[0]+boundary[1])/2<=110)
		{ 
			model=1;
			trunangle=0.5;
		}
		else if((boundary[0]+boundary[1])/2<=220&&(boundary[0]+boundary[1])/2>110)
		{
			isruning=true;
		}
		else if((boundary[0]+boundary[1])/2>=220)
		{   model=1;
			trunangle=-0.5;
		}
	 }
	 boundary[0]=-1;                                                                        //³õÊ¼»¯boundary
	 boundary[1]=-1;
	 boundary[2]=-1;
	 boundary[3]=-1;
	 boundary[4]=-1;
	 boundary[5]=-1;	

	 }
	
	imshow("contours1",kTopCamera_result);                                            //ÏÔÊ¾Í¼Æ¬0.5S
	mtx_.unlock();
	waitKey(500);
	kTopCamera_result=0;
	kTopCamera_contours.clear();                                                    //Çå¿ÕÊý¾Ý
    kTopCamera_corners.clear();
    kTopCamera_corners1.clear();
    kTopCamera_cornersx.clear();
   } 
  	
} 

/*******************************************************************************************************/
//¿ªÊ¼Ò»¸öÏß³Ì£¬ÓÃÀ´¶Ô»ñÈ¡ÏÂÃæÉãÏñÍ·Í¼ÏñºÍ´¦Àí·ÖÎöÍ¼Ïñ£¬ÅÐ¶ÏÇ°·½ÊÇ·ñÓÐÕÏ°­£¬²¢¼ì²âÊÇ·ñÓÐ¿ÕÎ»ÇøÓò
/****************************************************************************/

void kBottom_Camera()
{
   mtx_.lock();
   while(true)
   {
	  
   kBottomCamera_currentImage.data =0;                                               //Çå¿ÕkBottomCamera_currentImage¾ØÕóµÄÊý¾Ý
   kBottomCamera_curImage = videoProxy->getImageRemote(kBottomCamera_name);              //ÌáÈ¡ÉãÏñÍ·µÄËù¼ì²âµÄÖµ
   kBottomCamera_currentImage.data = (uchar*) kBottomCamera_curImage[6].GetBinary();      //¸³¸økBottomCamera_currentImage¾ØÕó
   
   threshold(kBottomCamera_currentImage,kBottomCamera_pBinaryImage,230,254,CV_THRESH_BINARY); //¶ÔÍ¼Ïñ½øÐÐ¶þÖµ»¯
   
   findContours(kBottomCamera_pBinaryImage, kBottomCamera_contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  //¼ì²âÂÖÀª£¬²¢¸³¸økBottomCamera_contoursÕâ¸ö¶¯Ì¬Êý×é
   drawContours(kBottomCamera_result,kBottomCamera_contours,-1,Scalar(255,0,0),1,8);            //ÃèÊöÂÖÀªÍ¼Ïñ£¬²¢¸³¸økBottomCamera_result¾ØÕó
   
   goodFeaturesToTrack(kBottomCamera_result,kBottomCamera_corners,cornersCount,0.05,30,Mat(),3,0,0.04);   //¼ì²âÂÖÀª½Çµã
   for (i=0;i<kBottomCamera_corners.size();i++)
       {
		 circle(kBottomCamera_result,cvPoint((int)(kBottomCamera_corners[i].x),(int)(kBottomCamera_corners[i].y)),5,CV_RGB(10,10,255),2,CV_AA,0);   //ÔÚÍ¼ÏñÖÐÈ¦×¡Ëù¼ì²âµ½µÄ½Çµã
		}
   
     
  
	for (i=0;i<kBottomCamera_corners.size();i++)
       {
		 kBottomCamera_cornersx.push_back(kBottomCamera_corners[i]);        //kBottom_contoursÕâ¸ö¶¯Ì¬Êý×éµÄÊý¾Ý¸´ÖÆµ½kBottomCamera_cornersx¶¯Ì¬Êý×é
		} 
	 
   for (i=0;i<kBottomCamera_corners.size()-1;i++)                       /***************************************************************/
	 {
	   
       for(int j=kBottomCamera_corners.size()-1;j>=i;j--)
	   {
		   if(j>0)
		   {
		    if((int)kBottomCamera_corners[j].y<(int)kBottomCamera_corners[j-1].y)          //½øÐÐ½ÇµãY×ø±êÃ°ÅÝÅÅÐò£¬²¢½«Êý¾Ý¸³¸økBottomCamera_corners
		     {
			   kBottomCamera_corners1.push_back(kBottomCamera_corners[j-1]);
			   kBottomCamera_corners[j-1]=kBottomCamera_corners[j];
			   kBottomCamera_corners[j]=kBottomCamera_corners1[0];
			   
			   kBottomCamera_corners1.pop_back();
			   
		     }
			
		    if((int)kBottomCamera_cornersx[j].x<(int)kBottomCamera_cornersx[j-1].x)       //½øÐÐ½ÇµãX×ø±êÃ°ÅÝÅÅÐò£¬²¢½«Êý¾Ý¸³¸økBottomCamera_cornersx
		     {
			   kBottomCamera_corners1.push_back(kBottomCamera_cornersx[j-1]);
			   kBottomCamera_cornersx[j-1]=kBottomCamera_cornersx[j];
			   kBottomCamera_cornersx[j]=kBottomCamera_corners1[0];
			   
			   kBottomCamera_corners1.pop_back();
			   
		     }
		   }
		 else 
			 break;
	   }
	 }                                                     /***************************************************************/

     std::cout<<std::endl;                                    //¼ì²âÍ¼ÏñÔÚy=170ÒÔÏÂÊÇ·ñ³öÏÖ½Çµã£¬Èç¹û³öÏÖÕÏ°­ÎïÔÚÃæÇ°£¬ÔòÐèµ÷Õû
	 for (i=0;i<kBottomCamera_cornersx.size();i++)
       {
		   int a=(int)kBottomCamera_cornersx[i].y;
		   //int b=(int)kBottomCamera_cornersx[i].x;
         if(a<=225&&a>=170)
		 {
			 isruning=false;                   //Í£Ö¹»úÆ÷ÈËÇ°½ø
			 
		 }
		// std::cout<<(int)kBottomCamera_corners[i].x<<"\t"<<(int)kBottomCamera_corners[i].y<<"\t"<<(int)kBottomCamera_cornersx[i].x<<"\t"<<(int)kBottomCamera_cornersx[i].y<<std::endl;
		}
	
	
	 int b=0;
	 if(isruning==false)                                                      //µ±¼ì²âµ½ÕÏ°­ÎïÔÚÇ°Ãæ£¬Ôò½øÐÐËÑË÷Í¼ÏñÖÐÊÇ·ñÓÐ¿ÕÎ»ÇøÓò£¨¿ÉÒÔÓÐ¼¸¸ö£©
	 {
	 for (i=0;i<kBottomCamera_cornersx.size()-1;i++)
       {                                                                      
        int c =(int)kBottomCamera_cornersx[i+1].x-(int)kBottomCamera_cornersx[i].x;    //Í¨¹ýÅÅÐòºóµÄ½Çµã×ø±êXÏà¼õ£¬ÔòÈ·¶¨ÊÇ·ñ´æÔÚ´óÐ¡ÒªÇóÎª120¡ª¡ª290µÄ¿ÕÎ»ÇøÓò
	    if(c>=120&&c<=290)                                                       //¿ÕÎ»ÇøÓòµÄ´óÐ¡ÒªÇóÎª120¡ª¡ª290
		{
		boundary[b]=(int)kBottomCamera_cornersx[i].x;                               //Èç¹û¼ì²âµ½ÓÐ£¬Ôò°Ñ¿ÕÎ»ÇøÓòµÄÎ»ÖÃ¸øboundaryÊý×éÒÔ±ãÓÃÉÏÃæÉãÏñÍ·À´È·¶¨ÕâÇøÓòÊÇ·ñ¿ÉÐÐ
		b=b+1;
		boundary[b]=(int)kBottomCamera_cornersx[i+1].x;
		b=b+1;

		}
		
	   } 
	 if(boundary[0]==-1)                                                        //Èç¹ûÃ»¼ì²âµ½£¬Ôò×ªÍä£¬model=1
	 {
		 model=1;
		
	 }
	 }                                                    /****************************************************************************/
	imshow("contours2",kBottomCamera_result);                  //ÏÔÊ¾Í¼Ïñ0.5s                                  
	mtx_.unlock();
	waitKey(500);
	kBottomCamera_result=0;
    kBottomCamera_contours.clear();                           //Çå¿ÕÊý¾Ý
    kBottomCamera_corners.clear();
    kBottomCamera_corners1.clear();
    kBottomCamera_cornersx.clear();
   } 
  	
} 


/*******************************************************************************************/
//¿ªÊ¼Ò»¸öÏß³Ì£¬¶Ô»úÆ÷ÈËÔË¶¯½øÐÐµ÷Õû

/**********************************************************************************************/

void run ()
{
 while(true)
 {
   while(isruning)                                       //ÔÚkBottom_CameraÏß³ÌÖÐÃ»ÓÐ¼ì²âµ½ÕÏ°­£¬ÔòÊµÐÐ¸ÃÑ­»·
   {
   mtx_.lock();
   motionProxy->moveToward(0.8,0.0,0.0);
   mtx_.unlock();
   }
   while(!isruning)                                     //ÔÚkBottom_CameraÏß³ÌÖÐ¼ì²âµ½ÕÏ°­£¬ÔòÊµÐÐ¸ÃÑ­»·
   {
  
   mtx_.lock();
   switch(model)
   {
	   case 1:
		   motionProxy->moveTo(0.0,0.0,0.5);
		   isruning=true;
		   mtx_.unlock();
		   break;
	   case 2:
		   motionProxy->moveTo(0.0,-distance,0.0);
		    isruning=true;
			distance=0;
			mtx_.unlock();
		   break;
	   case 3:
		   motionProxy->moveTo(0.0,distance,0.0);
		    isruning=true;
			distance=0;
			mtx_.unlock();
		   break;

   }

   }
 }
}

/****************************************************************************/
//main º¯Êý
/****************************************************************************/
int main(int argc, char*argv[])
{
  argc=2;
  if (argc != 2) {
    std::cerr << "Usage: alvisualcompass_example robotIp" << std::endl;
    return 1;
  }

  boost::shared_ptr<AL::ALBroker> broker = AL::ALBroker::createBroker("broker","", 0, "192.168.0.115", 9559, 0);


 

   try {
    videoProxy = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(broker));
    
    memoryProxy = boost::shared_ptr<ALMemoryProxy>(new ALMemoryProxy(broker));
	motionProxy = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy(broker));
      }
  catch (const ALError& e) {
    std::cerr << "Could not create proxies: " << e.what() << std::endl;
    return 2;
  }

   motionProxy->wakeUp();

   kTopCamera_name =videoProxy->subscribeCamera("kTopCamera",0, kQVGA, kYuvColorSpace, 30);
   kBottomCamera_name =videoProxy->subscribeCamera("kBottomCamera",1, kQVGA, kYuvColorSpace, 30);
 
   
   boost::thread thread1(kTop_Camera);         //¿ªÊ¼Ïß³Ì1
   boost::thread thread2(kBottom_Camera);       //¿ªÊ¼Ïß³Ì2
   boost::thread thread3(run);                 //¿ªÊ¼Ïß³Ì3
   thread1.join();                       //µÈ´ýÏß³Ì1½áÊø
   thread2.join();
   thread3.join();
 
   return 0;
}    
 



























	

