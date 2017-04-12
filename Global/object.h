/*
 * object.h
 *
 *  Created on: 31 марта 2017 г.
 *      Author: hudienko_a
 */

#ifndef GLOBAL_OBJECT_H_
#define GLOBAL_OBJECT_H_


class Object
{
public:
	Object(){}
	virtual ~Object(){}

	virtual void Init()=0;

	virtual const char* toString()=0;

private:
};



#endif /* GLOBAL_OBJECT_H_ */
