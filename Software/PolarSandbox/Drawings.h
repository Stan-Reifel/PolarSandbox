//      ******************************************************************
//      *                                                                *
//      *                   Header file for Drawings.c                   *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co,  2019              *
//      *                                                                *
//      ******************************************************************


#ifndef Drawings_h
#define Drawings_h

extern float finalThetaFromLastPlot;

//
// function declarations
//
extern int drawSpiral(boolean direction);
extern int drawSpiralHD(boolean direction);
extern int drawLooseSpiral(boolean direction);
extern int drawPetals(boolean direction);
extern int drawTriangles(boolean direction);
extern int drawTrianglesClipped(boolean direction);
extern int drawSquares(boolean direction);
extern int drawSquaresClipped(boolean direction);
extern int drawHexagons(boolean direction);
extern int drawPolygonSuperTwist(boolean direction);
extern int drawClovers(boolean direction);
extern int drawCloversTwisted(boolean direction);
extern int drawCloversSuperTwist(boolean direction);
extern int drawScalopsTwisted(boolean direction);
extern int drawScalopsTwistedClipped(boolean direction);
extern int drawLittleClovers(boolean direction);
extern int drawLittleCloversTwisted(boolean direction);
extern int drawStars(void);
extern int drawStarsTwisted(boolean direction);
extern int drawSpiderWeb(boolean direction);
extern int drawStarFlower(boolean direction);
extern int drawSunRays(boolean startEndPosition);
extern int drawHotSun(boolean startEndPosition);
extern int drawPsycho(boolean startEndPosition);
extern int drawSpiralRays(boolean startEndPosition);
extern int drawMoveSandInward(void);
extern int drawSixBlade(void);
extern int drawLoops(void);
extern int drawHearts(void);
extern int drawPerimeterSkirt(void);
extern int drawNarrowingSpirograph(void);
extern int drawScalopsTripleTwist(boolean direction);
extern int drawScalopsTripleTwistClipped(boolean direction);
extern int drawCloversTripleTwistClipped(boolean direction);


// ------------------------------------ End ---------------------------------
#endif
