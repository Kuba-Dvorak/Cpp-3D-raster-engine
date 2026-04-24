#include <iostream>
#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <cmath>
#include <regex>
#include <algorithm>
#include <array>
#include <functional>

#define MonitorHeight 1080
#define MonitorWidth 1920

const Uint8* state = SDL_GetKeyboardState(NULL);

//fully operational 3D raycasting engine

double getRandomDouble(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    double returnableValue = double((dis(gen)*(max-min+1))+min);
    return returnableValue;
}

int getRandomInt(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    int returnableValue = int((dis(gen)*(max-min+1))+min);
    return returnableValue;
}


struct simple3D_Pos_Double {
    double x, y, z;

    simple3D_Pos_Double(double x = 0, double y = 0, double z = 0) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    simple3D_Pos_Double changedBy(double changerX, double changerY, double changerZ) {
        return simple3D_Pos_Double(x + changerX, y + changerY, z + changerZ);
    }
};


class Vector3D_Double {
private:
    double absoluteLenght;
    simple3D_Pos_Double myPos;

public:

    Vector3D_Double(simple3D_Pos_Double impPos) {
        this->myPos = impPos;
        this->absoluteLenght = std::sqrt((myPos.x * myPos.x) + (myPos.y * myPos.y) + (myPos.z * myPos.z));
    }

    simple3D_Pos_Double getVec() {
        return myPos;
    }

    simple3D_Pos_Double& getVecRef() {
        return myPos;
    }

    double dotProduct(Vector3D_Double &secondVec) {
        return (secondVec.getVecRef().x * myPos.x) + (secondVec.getVecRef().y * myPos.y) + (secondVec.getVecRef().z * myPos.z);
    }

    void setVector(simple3D_Pos_Double impPos) {
        myPos.x = impPos.x;
        myPos.y = impPos.y;
        myPos.z = impPos.z;
        absoluteLenght = std::sqrt((myPos.x * myPos.x) + (myPos.y * myPos.y) + (myPos.z * myPos.z));
    }

    double absoluteValue() {
        return absoluteLenght;
    }

    double crossProduct2D(Vector3D_Double &secondVec) {
        return  myPos.x * secondVec.getVecRef().y - myPos.y * secondVec.getVecRef().x;
    }
};


struct SimpleColor {
    int red, blue, green, transp;
    SimpleColor(int red = 0, int green = 0, int blue = 0, int transp = 0) {
        this->red = red;
        this->blue = blue;
        this->green = green;
        this->transp = transp;
    }
    uint32_t convertToBinary() {
        uint32_t finalColor = 0x00000000;
        finalColor |= transp << 24;
        finalColor |= red << 16;
        finalColor |= green << 8;
        finalColor |= blue;
        return finalColor;
    }
};


struct screenAndCameraInfo {
    double numberAmpX, numberAmpY;
    int screenHeight, screenWidth;

    screenAndCameraInfo(double numberAmpX = 0, double numberAmpY = 0, int screenHeight = 0, int screenWidth = 0) {
        this->numberAmpX = numberAmpX;
        this->numberAmpY = numberAmpY;
        this->screenHeight = screenHeight;
        this->screenWidth = screenWidth;
    }
};


class Position3D_Double {
public:
    
    simple3D_Pos_Double myPos;

    Position3D_Double(simple3D_Pos_Double impPos = simple3D_Pos_Double(0,0,0)) {
        this->myPos.x = impPos.x;
        this->myPos.y = impPos.y;
        this->myPos.z = impPos.z;
    }

    simple3D_Pos_Double getPos() {
        return myPos;
    }
    

    void setPosition(simple3D_Pos_Double impPos) {
        myPos.x = impPos.x;
        myPos.y = impPos.y;
        myPos.z = impPos.z;
    }

    Vector3D_Double makeAVector(Position3D_Double &secondPos) {
        return Vector3D_Double(simple3D_Pos_Double(secondPos.myPos.x - myPos.x,secondPos.myPos.y - myPos.y,secondPos.myPos.z - myPos.z));
    }
    
    Vector3D_Double makeAUnitVector(Position3D_Double &secondPos) {
        double distance = std::sqrt(std::pow(myPos.x - secondPos.myPos.x, 2) + std::pow(myPos.y - secondPos.myPos.y, 2) + std::pow(myPos.z - secondPos.myPos.z, 2));

        if (distance < 0.05) {
            return Vector3D_Double(simple3D_Pos_Double(0,0,0));
        }

        return Vector3D_Double(simple3D_Pos_Double((secondPos.myPos.x - myPos.x) / distance,(secondPos.myPos.y - myPos.y) / distance,(secondPos.myPos.z - myPos.z) / distance));
    }

    Vector3D_Double makeA2DVector(Position3D_Double &secondPos) {
        return Vector3D_Double(simple3D_Pos_Double(myPos.x - secondPos.myPos.x,myPos.y - secondPos.myPos.y,0));
    }

    Position3D_Double changedBy(simple3D_Pos_Double changePos) {
        return Position3D_Double(simple3D_Pos_Double(myPos.x + changePos.x, myPos.y + changePos.y, myPos.z + changePos.z));
    }

    Position3D_Double makeIntoScreensCord(screenAndCameraInfo const &impInfo) {
        if (myPos.z > 0.1) {
            return Position3D_Double(simple3D_Pos_Double(std::round(((myPos.x / myPos.z) * impInfo.numberAmpX) + (impInfo.screenWidth / 2)), std::round(((myPos.y / myPos.z) * impInfo.numberAmpY) + (impInfo.screenHeight / 2)), myPos.z));
        }
        return Position3D_Double(simple3D_Pos_Double(-10, -10, -15));
    }
};


std::array<double, 2> getGradiantsDouble(Position3D_Double &pointA, Position3D_Double &pointB, Position3D_Double &pointC) {
    double diffX1 = pointB.myPos.x - pointA.myPos.x;
    double diffX2 = pointC.myPos.x - pointB.myPos.x;
    double diffY1 = pointB.myPos.y - pointA.myPos.y;
    double diffY2 = pointC.myPos.y - pointB.myPos.y;
    double diffZ1 = pointB.myPos.z - pointA.myPos.z;
    double diffZ2 = pointC.myPos.z - pointA.myPos.z;
    double determinant = diffX1 * diffY2 - diffX2 * diffY1;
    if (std::abs(determinant) <= 0.01) {
        determinant = 0.02;
    }
    double gradiantX = (diffY2 * diffZ1 - diffZ2 * diffY1) / (determinant);
    double gradiantY = (diffX1 * diffZ2 - diffZ1 * diffX2) / (determinant);
    return {gradiantX, gradiantY};
}


class ScreenPolygon_Double {
private:
    std::array<Position3D_Double, 3> points = {};
    SimpleColor myColor = {};

    std::array<int, 4> minsAndMaxs(screenAndCameraInfo const &camerasInfo) {
        std::array<int, 4> minsAMax = {int(points[0].myPos.x), int(points[0].myPos.y), int(points[0].myPos.x), int(points[0].myPos.y)};
        for (int i = 0; i < 3; i += 1) {
            if (points[i].myPos.x >= minsAMax[0]) {
                minsAMax[0] = int(points[i].myPos.x);
            }
            if (points[i].myPos.y >= minsAMax[1]) {
                minsAMax[1] = int(points[i].myPos.y);
            }

            if (points[i].myPos.x <= minsAMax[2]) {
                minsAMax[2] = int(points[i].myPos.x);
            }
            if (points[i].myPos.y <= minsAMax[3]) {
                minsAMax[3] = int(points[i].myPos.y);
            }
        }

        //border pripady

        if (minsAMax[2] <= 0) {
            minsAMax[2] = 0;
        }
        if (minsAMax[3] <= 0) {
            minsAMax[3] = 0;
        }
        if (minsAMax[0] >= camerasInfo.screenWidth) {
            minsAMax[0] = camerasInfo.screenWidth;
        }
        if (minsAMax[1] >= camerasInfo.screenHeight) {
            minsAMax[1] = camerasInfo.screenHeight;
        }
        return minsAMax;
    }

public:

    ScreenPolygon_Double(Position3D_Double point1, Position3D_Double point2, Position3D_Double point3, SimpleColor impCol) {
        this->points[0] = point1;
        this->points[1] = point2;
        this->points[2] = point3;
        this->myColor = impCol;
    }

    void changeColor(SimpleColor const &impCol) {
        myColor.red = impCol.red;
        myColor.green = impCol.green;
        myColor.blue = impCol.blue;
    }

    void drawOutPolygonQuickSDL2(double* zBufferImp, uint32_t* colorsBuffer, int pitch, screenAndCameraInfo const &cameraInfo, bool outLine, SimpleColor outLineCol) {

        std::array<int, 4> minsAmaxs = minsAndMaxs(cameraInfo);
        std::array<double, 2> gradiant = getGradiantsDouble(points[0], points[1], points[2]);

        uint32_t convertedColor = myColor.convertToBinary();
        uint32_t convetedOutLine = outLineCol.convertToBinary();

        double sizer = (minsAmaxs[0] - minsAmaxs[2]) * (minsAmaxs[1] - minsAmaxs[3])/2;
        double outLineBoundary = 1 * sqrt(sizer);
        
        double middleZ = (points[0].myPos.z + points[1].myPos.z + points[2].myPos.z) / 3;

        int blockyfacion = int(1 * sqrt(sizer) / middleZ);

        if (blockyfacion <= 0.5) {
            blockyfacion = 1;
        }


        for (int yPos = minsAmaxs[3]; yPos < minsAmaxs[1]; yPos += blockyfacion) {
            for (int xPos = minsAmaxs[2]; xPos < minsAmaxs[0]; xPos += blockyfacion) {

                Position3D_Double myPos = (simple3D_Pos_Double(xPos, yPos, 0));

                uint32_t pixelColor = convertedColor;

                Vector3D_Double vectorSA = points[0].makeA2DVector(myPos);
                Vector3D_Double vectorSB = points[1].makeA2DVector(myPos);
                double abCrossProd = vectorSA.crossProduct2D(vectorSB);

                if (abCrossProd < 0) {
                    continue;
                }

                Vector3D_Double vectorSC = points[2].makeA2DVector(myPos);
                double bcCrossProd = vectorSB.crossProduct2D(vectorSC);
                if (bcCrossProd < 0) {
                    continue;
                }

                double caCrossProd = vectorSC.crossProduct2D(vectorSA);
                if (caCrossProd < 0) {
                    continue;
                }

                if (outLine) {
                    if (abCrossProd < outLineBoundary || bcCrossProd < outLineBoundary || caCrossProd < outLineBoundary) {
                        pixelColor = convetedOutLine;
                    }
                }


                for (int yPos2 = yPos; yPos2 < yPos + blockyfacion; yPos2 += 1) {

                    if (yPos2 >= cameraInfo.screenHeight) {
                        yPos2 += blockyfacion;
                        continue;
                    }

                    for (int xPos2 = xPos; xPos2 < xPos + blockyfacion; xPos2 += 1) {

                        if (xPos2 >= cameraInfo.screenWidth) {
                            xPos2 += blockyfacion;
                            continue;
                        }

                        double globalZ = points[2].myPos.z + (xPos2 - points[2].myPos.x) * gradiant[0] + (yPos2 - points[2].myPos.y) * gradiant[1];

                        if (globalZ < zBufferImp[xPos2 + (yPos2 * cameraInfo.screenWidth)] ) {
                            colorsBuffer[xPos2 + (yPos2 * (pitch / 4))] = pixelColor;
                            zBufferImp[xPos2 + (yPos2 * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }
    }

    void drawOutPolygonDouble(double* zBufferImp, SimpleColor* colorsBuffer, screenAndCameraInfo const &cameraInfo, bool outLine, SimpleColor outLineCol) {
        
        std::array<int, 4> minsAmaxs = minsAndMaxs(cameraInfo);
        std::array<double, 2> gradiant = getGradiantsDouble(points[0], points[1], points[2]);
        
        double outLineBoundary = (minsAmaxs[0] - minsAmaxs[2]) * (minsAmaxs[1] - minsAmaxs[3]) * 0.01;

        for (int yPos = minsAmaxs[3]; yPos < minsAmaxs[1]; yPos += 1) {
            for (int xPos = minsAmaxs[2]; xPos < minsAmaxs[0]; xPos += 1) {

                Position3D_Double myPos = (simple3D_Pos_Double(xPos, yPos, 0));

                SimpleColor pixelColor = myColor;

                Vector3D_Double vectorSA = points[0].makeA2DVector(myPos);
                Vector3D_Double vectorSB = points[1].makeA2DVector(myPos);
                double abCrossProd = vectorSA.crossProduct2D(vectorSB);

                if (abCrossProd < 0) {
                    continue;
                }

                Vector3D_Double vectorSC = points[2].makeA2DVector(myPos);
                double bcCrossProd = vectorSB.crossProduct2D(vectorSC);
                if (bcCrossProd < 0) {
                    continue;
                }

                double caCrossProd = vectorSC.crossProduct2D(vectorSA);
                if (caCrossProd < 0) {
                    continue;
                }

                if (outLine) {
                    if (abCrossProd < outLineBoundary || bcCrossProd < outLineBoundary || caCrossProd < outLineBoundary) {
                        pixelColor.blue = outLineCol.blue;
                        pixelColor.green = outLineCol.green;
                        pixelColor.red = outLineCol.red;
                    }
                }

                double globalZ = points[2].myPos.z + (xPos - points[2].myPos.x) * gradiant[0] + (yPos - points[2].myPos.y) * gradiant[1];
                if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] ) {
                    colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = pixelColor;
                    zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] = globalZ;
                }
            }
        }
    }

};


class Point_Double {
private:
    Position3D_Double position = {};
public:

    Point_Double(simple3D_Pos_Double impPosition) {
        this->position.myPos = impPosition;
    }

    Position3D_Double getPos() {
        return position;
    }

    void setPos(simple3D_Pos_Double changePos) {
        position.myPos = changePos;
    }

    Position3D_Double getRelativePos(Vector3D_Double &headingVec, Vector3D_Double &upVector, Vector3D_Double &rightVector, Position3D_Double &headingOrigin) {

        Vector3D_Double vectorOS = headingOrigin.makeAVector(position);

        return Position3D_Double(simple3D_Pos_Double(vectorOS.dotProduct(rightVector), vectorOS.dotProduct(upVector), vectorOS.dotProduct(headingVec)));
    }
};


class Object3D_Double {
private:
    // links in style of having xxx and then another xxx ...

    std::vector<int> links;
    std::vector<Point_Double> points;
    simple3D_Pos_Double centrePoint;
    SimpleColor objectColor;
    int numsOfPoints;
    int numsOfFaces;
    bool stroked;
    SimpleColor outlineCol;
    simple3D_Pos_Double objectSize;
    std::vector<Vector3D_Double> vectorsFromCentre;

public:

    Object3D_Double(int numOfPoints, int numFaces, std::vector<Point_Double> &points, simple3D_Pos_Double &centre, std::vector<int> impLinks, simple3D_Pos_Double impSize, SimpleColor objectColor, SimpleColor outlineCol, bool stroked) {
        this->stroked = stroked;
        this->outlineCol = outlineCol;
        this->objectColor = objectColor;
        this->numsOfFaces = numFaces;
        this->numsOfPoints = numOfPoints;
        this->centrePoint = centre;
        this->objectSize = impSize;
        this->links = impLinks;

        Position3D_Double centrePointPosition = Position3D_Double(centrePoint);

        for (int i = 0; i < numOfPoints; i += 1) {
            this->points.push_back(points[i]);

            Position3D_Double onePos = points[i].getPos();
            this->vectorsFromCentre.push_back(centrePointPosition.makeAUnitVector(onePos));
        }
    }

    void rotates(double angleY, double angleZ, double angleX) {
        double cosAngle = std::cos(angleY);
        double sinAngle = std::sin(angleY);

        for (int i = 0; i < numsOfPoints; i += 1) {

            simple3D_Pos_Double diff = simple3D_Pos_Double();

        }

    }

    simple3D_Pos_Double getPos() {
        return centrePoint;
    }

    void setPos(simple3D_Pos_Double changePos) {

        centrePoint = changePos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            points[i].setPos(changePos);
        }
    }

    void setSize(simple3D_Pos_Double newSize) {
        for (int i = 0; i < numsOfPoints; i += 1) {
            simple3D_Pos_Double oneVec =  vectorsFromCentre[i].getVec();
            
            points[i].setPos(simple3D_Pos_Double(oneVec.x * newSize.x, oneVec.y * newSize.y, oneVec.z * newSize.z));
        }
    }

    void drawOut(Vector3D_Double &headingVec, Vector3D_Double &upVector, Vector3D_Double &rightVector, Position3D_Double &headingOrigin, screenAndCameraInfo &camera_info, SimpleColor* colorsBuffer, double *zBuffer) {

        std::vector<Position3D_Double> pseudoPos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            pseudoPos[i] = (points[i].getRelativePos(headingVec, upVector, rightVector, headingOrigin)).makeIntoScreensCord(camera_info);
        }

        for (int i = 0; i < numsOfFaces; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 || pseudoPos[links[i+1]].myPos.z < 0.1 || pseudoPos[links[i+2]].myPos.z < 0.1) {
                continue;
            }

            ScreenPolygon_Double onePolygon = ScreenPolygon_Double(pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]], objectColor);
            onePolygon.drawOutPolygonDouble(zBuffer, colorsBuffer, camera_info, stroked, outlineCol);
        }
    }

    void drawOutSDL2(Vector3D_Double &headingVec, Vector3D_Double &upVector, Vector3D_Double &rightVector, Position3D_Double &headingOrigin, screenAndCameraInfo &camera_info, uint32_t* colorsBuffer, int pitch, double *zBuffer) {

        std::vector<Position3D_Double> pseudoPos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            pseudoPos.push_back((points[i].getRelativePos(headingVec, upVector, rightVector, headingOrigin)).makeIntoScreensCord(camera_info));
        }

        for (int i = 0; i < numsOfFaces; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 || pseudoPos[links[i+1]].myPos.z < 0.1 || pseudoPos[links[i+2]].myPos.z < 0.1) {
                continue;
            }

            ScreenPolygon_Double onePolygon = ScreenPolygon_Double(pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]], objectColor);
            onePolygon.drawOutPolygonQuickSDL2(zBuffer, colorsBuffer, pitch, camera_info, stroked, outlineCol);
        }
    }
};


struct playerHelpfulVals {
    double sinAngleY, cosAngleY, cosAngleZ, sinAngleZ;
    double angleY, angleZ;
    
    playerHelpfulVals(double angleY = 0, double angleZ = 0) {
        this->angleY = angleY;
        this->angleZ = angleZ;
        this->sinAngleY = std::sin(angleY);
        this->cosAngleY = std::cos(angleY);
        this->sinAngleZ = std::sin(angleZ);
        this->cosAngleZ = std::cos(angleZ);
    }
    
    void changeAngleY(double changeY) {
        angleY += changeY;
        sinAngleY = std::sin(angleY);
        cosAngleY = std::cos(angleY);
    }
    
    void changeAngleZ(double changeZ) {
        angleZ += changeZ;
        sinAngleZ = std::sin(angleZ);
        cosAngleZ = std::cos(angleZ);
    }
};


Object3D_Double createCubePoints(simple3D_Pos_Double onePos, simple3D_Pos_Double oneSize, SimpleColor objColor, SimpleColor outColor, bool stroked) {
    std::vector<Point_Double> points;
    points.push_back(Point_Double(onePos.changedBy(0, 0, 0)));
    points.push_back(Point_Double(onePos.changedBy(oneSize.x, 0, 0)));
    points.push_back(Point_Double(onePos.changedBy(oneSize.x, oneSize.y, 0)));
    points.push_back(Point_Double(onePos.changedBy(oneSize.x, oneSize.y, oneSize.z)));
    points.push_back(Point_Double(onePos.changedBy(0, oneSize.y, oneSize.z)));
    points.push_back(Point_Double(onePos.changedBy(0, 0, oneSize.z)));
    points.push_back(Point_Double(onePos.changedBy(0, oneSize.y, 0)));
    points.push_back(Point_Double(onePos.changedBy(oneSize.x, 0, oneSize.z)));

    simple3D_Pos_Double centre = simple3D_Pos_Double(onePos.x + (oneSize.x/2), onePos.y + (oneSize.y/2), onePos.z + (oneSize.z/2));

    return Object3D_Double(8, 36, points, centre, {
        0,2,1, 0,6,2,  // Z=0
         3,4,5, 3,5,7,  // Z=1
         0,5,4, 0,4,6,  // X=0
         1,2,3, 1,3,7,  // X=1
         0,1,7, 0,7,5,  // Y=0
         2,4,3, 2,6,4   // Y=1
    }, oneSize, objColor, outColor, stroked);
}


class Player_Double {
private:
    screenAndCameraInfo myCameraInfo;
    double speed;
    playerHelpfulVals values;
    playerHelpfulVals valuesUp;
    playerHelpfulVals valuesRight;
    Vector3D_Double headingVec = Vector3D_Double(simple3D_Pos_Double());
    Vector3D_Double rightVec = Vector3D_Double(simple3D_Pos_Double());
    Vector3D_Double upVec = Vector3D_Double(simple3D_Pos_Double());
    Position3D_Double myPos = Position3D_Double(simple3D_Pos_Double());
public:

    Player_Double(double speed = 0.5, int screenHeight = 1080, int screenWidth = 1920, int fov = 90, simple3D_Pos_Double beginPos = simple3D_Pos_Double(0,0,0)) {
        this->myPos.setPosition(beginPos);
        this->speed = speed;
        
        this->values = playerHelpfulVals(0,0);
        this->valuesUp = playerHelpfulVals(this->values.angleY,(this->values.angleZ - M_PI / 2));
        this->valuesRight = playerHelpfulVals((this->values.angleY + M_PI / 2),this->values.angleZ);
        
        this->headingVec.setVector(simple3D_Pos_Double(this->values.cosAngleY * this->values.cosAngleZ, this->values.sinAngleY * this->values.cosAngleZ, this->values.sinAngleZ));
        this->rightVec.setVector(simple3D_Pos_Double(this->valuesRight.cosAngleY * this->valuesRight.cosAngleZ, this->valuesRight.sinAngleY * this->valuesRight.cosAngleZ, 0));
        this->upVec.setVector(simple3D_Pos_Double(this->valuesUp.cosAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleZ));
        
        double radiansFOV = fov * (M_PI / 180);
        this->myCameraInfo = screenAndCameraInfo(((screenWidth / 2) / (std::tan(radiansFOV/2))) * 1.7, ((screenHeight / 2) / (std::tan(radiansFOV/2))), screenHeight, screenWidth);
    }

    void camera(std::vector<Object3D_Double> const &objects, uint32_t* sdlBuffer, double* zBuffer, SimpleColor* universalColorBuffer, bool sdl2Type, int pitch) {
        if (sdl2Type) {
            for (Object3D_Double oneObject : objects) {
                oneObject.drawOutSDL2(headingVec, upVec, rightVec, myPos, myCameraInfo, sdlBuffer, pitch, zBuffer);
            }
        }
        if (!sdl2Type) {
            for (Object3D_Double oneObject : objects) {
                oneObject.drawOut(headingVec, upVec, rightVec, myPos, myCameraInfo, universalColorBuffer, zBuffer);
            }
        }
    }

    void movement() {
        if (state[SDL_SCANCODE_W]) {
            myPos.myPos.x += speed * values.cosAngleY;
            myPos.myPos.y += speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_S]) {
            myPos.myPos.x -= speed * values.cosAngleY;
            myPos.myPos.y -= speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_D]) {
            myPos.myPos.x += speed * valuesRight.cosAngleY;
            myPos.myPos.y += speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_A]) {
            myPos.myPos.x -= speed * valuesRight.cosAngleY;
            myPos.myPos.y -= speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_SPACE]) {
            myPos.myPos.z += speed;
        }
        if (state[SDL_SCANCODE_LSHIFT]) {
            myPos.myPos.z -= speed;
        }
        if (state[SDL_SCANCODE_DOWN]) {
            if (values.angleZ >= -1) {
                values.changeAngleZ(-0.05);
                valuesUp.changeAngleZ(-0.05);
                
                headingVec.setVector(simple3D_Pos_Double(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
                rightVec.setVector(simple3D_Pos_Double(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
                upVec.setVector(simple3D_Pos_Double(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
            }
        }
        if (state[SDL_SCANCODE_UP]) {
            if (values.angleZ <= 1) {
                values.changeAngleZ(0.05);
                valuesUp.changeAngleZ(0.05);

                headingVec.setVector(simple3D_Pos_Double(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
                rightVec.setVector(simple3D_Pos_Double(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
                upVec.setVector(simple3D_Pos_Double(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
            }
        }
        if (state[SDL_SCANCODE_LEFT]) {
            values.changeAngleY(-0.05);
            valuesUp.changeAngleY(-0.05);
            valuesRight.changeAngleY(-0.05);

            headingVec.setVector(simple3D_Pos_Double(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_Double(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_Double(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
        }
        if (state[SDL_SCANCODE_RIGHT]) {
            values.changeAngleY(0.05);
            valuesUp.changeAngleY(0.05);
            valuesRight.changeAngleY(0.05);

            headingVec.setVector(simple3D_Pos_Double(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_Double(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_Double(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
        }
    }
};


uint32_t convertToBinary(int red, int green, int blue) {
    uint32_t finalColor = 0xff000000;
    finalColor |= red << 16;
    finalColor |= green << 8;
    finalColor |= blue;
    return finalColor;
}


int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cout << "SDL se nepovedlo inicializovat (je to docela v pici): " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Hra 3D engine",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        MonitorWidth, MonitorHeight,
        SDL_WINDOW_OPENGL
    );

    if (!window) {
        std::cout << "Okno neslo vytvorit: " << SDL_GetError() << std::endl;
        return 1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    SDL_Texture* buffer = SDL_CreateTexture(renderer,
    SDL_PIXELFORMAT_ARGB8888,
    SDL_TEXTUREACCESS_STREAMING,
    MonitorWidth, MonitorHeight);


    bool running = true;
    SDL_Event event;

    Player_Double firstPlayer = Player_Double();

    std::vector<Object3D_Double> kostky = {};
    for (int i = 0; i < 10; i += 1) {
        kostky.push_back(createCubePoints(simple3D_Pos_Double(getRandomDouble(-35,35),getRandomDouble(-35,35),getRandomDouble(-35,35)), simple3D_Pos_Double(getRandomDouble(2,10),getRandomDouble(2,10),getRandomDouble(2,10)),
            SimpleColor(getRandomInt(1,255),getRandomInt(1,255),getRandomInt(1,255)), SimpleColor(0, 0, 0), true));
    }

    kostky.push_back(createCubePoints(simple3D_Pos_Double(5,10,3), simple3D_Pos_Double(2,3,1), SimpleColor(250,0,0), SimpleColor(0, 0, 0), false));
    kostky.push_back(createCubePoints(simple3D_Pos_Double(2,15,13), simple3D_Pos_Double(2,3,1), SimpleColor(250,0,0), SimpleColor(0, 0, 0), false));
    kostky.push_back(createCubePoints(simple3D_Pos_Double(12,15,13), simple3D_Pos_Double(2,3,3), SimpleColor(250,120,0), SimpleColor(0, 0, 0), false));

    double renderDistance = 200;

    uint32_t* nasPixelBuffer;
    double* zBuffer = nullptr;

    //unused SimpleBuffer
    SimpleColor* bufferColorMy = nullptr;

    zBuffer = (double*)malloc(MonitorHeight * MonitorWidth * sizeof(double));
    std::fill(zBuffer, zBuffer + MonitorHeight * MonitorWidth, renderDistance);
    int pitch;

    uint32_t backgroundColor = convertToBinary(0,90,200);


    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
            }
        }

        SDL_RenderClear(renderer);

        SDL_LockTexture(buffer, NULL, (void**)&nasPixelBuffer, &pitch);

        std::fill(nasPixelBuffer, nasPixelBuffer + MonitorHeight * MonitorWidth, backgroundColor);
        std::fill(zBuffer, zBuffer + MonitorHeight * MonitorWidth, renderDistance);

        firstPlayer.camera(kostky, nasPixelBuffer, zBuffer, bufferColorMy, true, pitch);

        SDL_UnlockTexture(buffer);
        SDL_RenderCopy(renderer, buffer, NULL, NULL);
        firstPlayer.movement();

        SDL_RenderPresent(renderer);
    }


    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::free(zBuffer);


    return 0;
}