#include <iostream>
#include "Windows.h" 
#include "app.h"

using namespace std;

int main(int argc,char * argv[])
{		

	cap a;
	//开始读取数据
   if (SUCCEEDED(a.ConnectKinect()))
   {	while( 1 ){

			a.getDdata();

			a.getSdata();

			if( cv::waitKey( 30 ) == VK_ESCAPE ){
			break;
			}
					}
	}
	return 0;
}
