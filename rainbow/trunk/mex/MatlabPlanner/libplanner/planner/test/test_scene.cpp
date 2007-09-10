#include "planner\sceneparser.h"

#include <stdio.h>
#include <time.h>

int main(void)
{

	XMLSceneData parser("D:\\projects\\rainbow\\trunk\\scene\\twopa10_fullbody.scene");
	parser.load();

}