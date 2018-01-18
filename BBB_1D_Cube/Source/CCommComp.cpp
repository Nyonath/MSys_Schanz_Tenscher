/*
 * CCommComp.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: Robin Tenscher
 *      		Tim Schanz
 */

#include "../Header/CCommComp.h"

CCommComp::CCommComp()
{

}

CCommComp::~CCommComp()
{
}
void CCommComp::run()
{
	std::cout << "Communication Comp" << std::endl;
	while (1)
	{
		bool ret = CCont.getContent(true, mScont);

		if (ret)
		{
			//mServer.transmitMessage(mScont);

		}

	}
}
void CCommComp::init()
{
	//mServer.init();
	//mServer.waitForClient();
}

