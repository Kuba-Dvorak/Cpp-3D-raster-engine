#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <regex>
#include <algorithm>
#include <array>

//fully operational 3D raycasting engine

float getRandomfloat(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    float returnableValue = float((dis(gen)*(max-min+1))+min);
    return returnableValue;
}

int getRandomInt(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    int returnableValue = int((dis(gen)*(max-min+1))+min);
    return returnableValue;
}


struct simple3D_Pos_float {
    float x, y, z;

    simple3D_Pos_float(float x = 0, float y = 0, float z = 0) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    simple3D_Pos_float changedBy(float changerX, float changerY, float changerZ) {
        return simple3D_Pos_float(x + changerX, y + changerY, z + changerZ);
    }
};


class Vector3D_float {
public:
    float absoluteLenght;
    simple3D_Pos_float myPos;

    Vector3D_float(simple3D_Pos_float impPos) {
        this->myPos = impPos;
        this->absoluteLenght = std::sqrt((myPos.x * myPos.x) + (myPos.y * myPos.y) + (myPos.z * myPos.z));
    }

    simple3D_Pos_float getVec() {
        return myPos;
    }

    simple3D_Pos_float& getVecRef() {
        return myPos;
    }

    float dotProduct(Vector3D_float &secondVec) {
        return (secondVec.getVecRef().x * myPos.x) + (secondVec.getVecRef().y * myPos.y) + (secondVec.getVecRef().z * myPos.z);
    }

    float dotProductSimple(simple3D_Pos_float &secondVec) {
        return (secondVec.x * myPos.x) + (secondVec.y * myPos.y) + (secondVec.z * myPos.z);
    }

    void setVector(simple3D_Pos_float impPos) {
        myPos.x = impPos.x;
        myPos.y = impPos.y;
        myPos.z = impPos.z;
        absoluteLenght = std::sqrt((myPos.x * myPos.x) + (myPos.y * myPos.y) + (myPos.z * myPos.z));
    }

    float absoluteValue() {
        return absoluteLenght;
    }

    float crossProduct2D(Vector3D_float &secondVec) {
        return  myPos.x * secondVec.getVecRef().y - myPos.y * secondVec.getVecRef().x;
    }
};


struct SimpleColor {
    int red, blue, green, transp;
    uint8_t myConvertedCol;

    SimpleColor(int red = 0, int green = 0, int blue = 0, int transp = 0) {
        this->red = red;
        this->blue = blue;
        this->green = green;
        this->transp = transp;
    }

    uint8_t convertToBinary() {
        uint8_t finalColor = 0b00000000;
        finalColor |= (red >> 6) << 4;
        finalColor |= (green >> 6) << 2;
        finalColor |= (blue >> 6);
        return finalColor;
    }
};


struct screenAndCameraInfo {
    float numberAmpX, numberAmpY;
    int screenHeight, screenWidth;

    screenAndCameraInfo(float numberAmpX = 0, float numberAmpY = 0, int screenHeight = 0, int screenWidth = 0) {
        this->numberAmpX = numberAmpX;
        this->numberAmpY = numberAmpY;
        this->screenHeight = screenHeight;
        this->screenWidth = screenWidth;
    }
};


class Position3D_float {
public:
    
    simple3D_Pos_float myPos;

    Position3D_float(simple3D_Pos_float impPos = simple3D_Pos_float(0,0,0)) {
        this->myPos.x = impPos.x;
        this->myPos.y = impPos.y;
        this->myPos.z = impPos.z;
    }

    simple3D_Pos_float getPos() {
        return myPos;
    }
    

    void setPosition(simple3D_Pos_float impPos) {
        myPos.x = impPos.x;
        myPos.y = impPos.y;
        myPos.z = impPos.z;
    }

    void changePosition(simple3D_Pos_float impPos) {
        myPos.x += impPos.x;
        myPos.y += impPos.y;
        myPos.z += impPos.z;
    }

    Vector3D_float makeAVector(Position3D_float &secondPos) {
        return Vector3D_float(simple3D_Pos_float(secondPos.myPos.x - myPos.x,secondPos.myPos.y - myPos.y,secondPos.myPos.z - myPos.z));
    }
    
    Vector3D_float makeAUnitVector(Position3D_float secondPos) {
        float distance = std::sqrt(std::pow(myPos.x - secondPos.myPos.x, 2) + std::pow(myPos.y - secondPos.myPos.y, 2) + std::pow(myPos.z - secondPos.myPos.z, 2));

        if (distance < 0.05) {
            distance = 0.06;
        }

        return Vector3D_float(simple3D_Pos_float((secondPos.myPos.x - myPos.x) / distance,(secondPos.myPos.y - myPos.y) / distance,(secondPos.myPos.z - myPos.z) / distance));
    }

    Vector3D_float makeA2DVector(Position3D_float &secondPos) {
        return Vector3D_float(simple3D_Pos_float(myPos.x - secondPos.myPos.x,myPos.y - secondPos.myPos.y,0));
    }

    float absoluteDistance(Position3D_float &secondPos) {
        return std::sqrt(std::pow(secondPos.myPos.x - myPos.x,2) + std::pow(secondPos.myPos.y - myPos.y,2) + std::pow(secondPos.myPos.z - myPos.z,2));
    }

    float absoluteDistanceSimple(simple3D_Pos_float &secondPos) {
        return std::sqrt(std::pow(secondPos.x - myPos.x,2) + std::pow(secondPos.y - myPos.y,2) + std::pow(secondPos.z - myPos.z,2));
    }

    Position3D_float changedBy(simple3D_Pos_float changePos) {
        return Position3D_float(simple3D_Pos_float(myPos.x + changePos.x, myPos.y + changePos.y, myPos.z + changePos.z));
    }

    Position3D_float makeIntoScreensCord(screenAndCameraInfo const &impInfo) {

        if (myPos.z > 0.01) {
            return Position3D_float(simple3D_Pos_float(std::round(((myPos.x / myPos.z) * impInfo.numberAmpX) + (impInfo.screenWidth / 2)), std::round(((myPos.y / myPos.z) * impInfo.numberAmpY) + (impInfo.screenHeight / 2)), myPos.z));
        }

        return Position3D_float(simple3D_Pos_float(-10, -10, -15));
    }
};


std::array<float, 2> getGradiantsfloat(Position3D_float &pointA, Position3D_float &pointB, Position3D_float &pointC) {
    float diffX1 = pointB.myPos.x - pointA.myPos.x;
    float diffX2 = pointC.myPos.x - pointB.myPos.x;
    float diffY1 = pointB.myPos.y - pointA.myPos.y;
    float diffY2 = pointC.myPos.y - pointB.myPos.y;
    float diffZ1 = pointB.myPos.z - pointA.myPos.z;
    float diffZ2 = pointC.myPos.z - pointB.myPos.z;
    float determinant = diffX1 * diffY2 - diffX2 * diffY1;
    if (std::abs(determinant) <= 0.01) {
        determinant = 0.02;
    }
    float gradiantX = (diffY2 * diffZ1 - diffZ2 * diffY1) / (determinant);
    float gradiantY = (diffX1 * diffZ2 - diffZ1 * diffX2) / (determinant);
    return {gradiantX, gradiantY};
}


class ScreenPolygon_float {
private:
    std::array<Position3D_float, 3> points = {};
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
            minsAMax[0] = camerasInfo.screenWidth-2;
        }
        if (minsAMax[1] >= camerasInfo.screenHeight) {
            minsAMax[1] = camerasInfo.screenHeight-2;
        }
        return minsAMax;
    }

    std::array<int, 3> minsAndMaxsAMiddle(bool &xMajority) {
        std::array<int, 3> minsAMax = {0,0,0};

        if (xMajority) {
            for (int i = 0; i < 3; i += 1) {
                if (points[i].myPos.y >= points[minsAMax[0]].myPos.y) {
                    minsAMax[0] = i;
                }
                if (points[i].myPos.y <= points[minsAMax[1]].myPos.y) {
                    minsAMax[1] = i;
                }
            }
            minsAMax[2] = (3 - minsAMax[0] - minsAMax[1]);

            if (minsAMax[2] > 2 || minsAMax[2] < 0) {
                minsAMax[2] = 2;
            }
        }


        else {
            for (int i = 0; i < 3; i += 1) {
                if (points[i].myPos.x >= points[minsAMax[0]].myPos.x) {
                    minsAMax[0] = i;
                }
                if (points[i].myPos.x <= points[minsAMax[1]].myPos.x) {
                    minsAMax[1] = i;
                }
            }
            minsAMax[2] = (3 - minsAMax[0] - minsAMax[1]);

            if (minsAMax[2] > 2 || minsAMax[2] < 0) {
                minsAMax[2] = 2;
            }
        }
        return minsAMax;
    }

    bool compareMinsAndMaxs(std::array<int, 4> const &infoArray) {
        if (std::abs(infoArray[0] - infoArray[2]) >= std::abs(infoArray[3] - infoArray[1])) {
            return true;
        }
        return false;
    }

    std::array<float, 3> getKoeficients(bool xMajority, std::array<int, 3> const &infos) {
        std::array<float, 3> returningKoeficients = {};

        float diveder0y = (points[infos[0]].myPos.y - points[infos[1]].myPos.y);
        float diveder1y = (points[infos[2]].myPos.y - points[infos[1]].myPos.y);
        float diveder2y = (points[infos[0]].myPos.y - points[infos[2]].myPos.y);

        if (diveder0y <= 0.01 && diveder0y >= 0) {
            diveder0y = 0.02;
        }

        if (diveder1y <= 0.01 && diveder1y >= 0) {
            diveder1y = 0.02;
        }

        if (diveder2y <= 0.01 && diveder2y >= 0) {
            diveder2y = 0.02;
        }

        if (diveder0y >= -0.01 && diveder0y <= 0) {
            diveder0y = -0.02;
        }

        if (diveder1y >= -0.01 && diveder1y <= 0) {
            diveder1y = -0.02;
        }

        if (diveder2y >= -0.01 && diveder2y <= 0) {
            diveder2y = -0.02;
        }

        float diveder0x = (points[infos[0]].myPos.x - points[infos[1]].myPos.x);
        float diveder1x = (points[infos[2]].myPos.x - points[infos[1]].myPos.x);
        float diveder2x = (points[infos[0]].myPos.x - points[infos[2]].myPos.x);

        if (diveder0x <= 0.01 && diveder0x >= 0) {
            diveder0x = 0.02;
        }

        if (diveder1x <= 0.01 && diveder1x >= 0) {
            diveder1x = 0.02;
        }

        if (diveder2x <= 0.01 && diveder2x >= 0) {
            diveder2x = 0.02;
        }

        if (diveder0x >= -0.01 && diveder0x <= 0) {
            diveder0x = -0.02;
        }

        if (diveder1x >= -0.01 && diveder1x <= 0) {
            diveder1x = -0.02;
        }

        if (diveder2x >= -0.01 && diveder2x <= 0) {
            diveder2x = -0.02;
        }

        if (xMajority) {

            returningKoeficients[0] = diveder0x / diveder0y;
            returningKoeficients[1] = diveder1x / diveder1y;
            returningKoeficients[2] = diveder2x / diveder2y;
        }

        else {

            returningKoeficients[0] = diveder0y / diveder0x;
            returningKoeficients[1] = diveder1y / diveder1x;
            returningKoeficients[2] = diveder2y / diveder2x;
        }
        return returningKoeficients;
    }

public:

    ScreenPolygon_float(Position3D_float point1, Position3D_float point2, Position3D_float point3, SimpleColor impCol) {
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

    void drawOutPolygonSDL2Fast(uint8_t* zBufferImp, uint8_t* colorsBuffer, screenAndCameraInfo const &cameraInfo, bool outLine, SimpleColor outLineCol, int outlineThickness, bool blockification, float blockDetail) {
        std::array<int, 4> minsAmaxs = minsAndMaxs(cameraInfo);
        std::array<float, 2> gradiant = getGradiantsfloat(points[0], points[1], points[2]);

        uint8_t convertedColor = myColor.convertToBinary();
        uint8_t convertedOutLine = outLineCol.convertToBinary();

        bool xMajority = compareMinsAndMaxs(minsAmaxs);

        std::array<int, 3> indexesNumbers = minsAndMaxsAMiddle(xMajority);

        std::array<float, 3> koeficients = getKoeficients(xMajority, indexesNumbers);

        float blockyfacion = 1;

        if (blockification) {
            float sizer = (minsAmaxs[0] - minsAmaxs[2]) * (minsAmaxs[1] - minsAmaxs[3]) / 2;
            float middleZ = (points[0].myPos.z + points[1].myPos.z + points[2].myPos.z) / 3;
            blockyfacion = int(blockDetail * sqrt(sizer) / middleZ);
        }

        if (blockyfacion <= 1) {
            blockyfacion = 1;
        }

        if (xMajority) {
            bool leftRight = false;
            int minX = 0;
            int maxX = 0;
            if (points[indexesNumbers[0]].myPos.x + (koeficients[0] * (minsAmaxs[3] - points[indexesNumbers[0]].myPos.y)) > points[indexesNumbers[0]].myPos.x + (koeficients[2] * (minsAmaxs[3] - points[indexesNumbers[0]].myPos.y))) {
                leftRight = true;
            }

            for (int yPos = minsAmaxs[3]; yPos < minsAmaxs[1]+1; yPos += blockyfacion) {
                int xPos2 = std::round(points[indexesNumbers[0]].myPos.x + (koeficients[0] * (yPos - points[indexesNumbers[0]].myPos.y)));
                int xPos1 = 0;
                if (yPos >= points[indexesNumbers[2]].myPos.y) {
                    xPos1 = std::round(points[indexesNumbers[0]].myPos.x + (koeficients[2] * (yPos - points[indexesNumbers[0]].myPos.y)));
                }

                else {
                    xPos1 = std::round(points[indexesNumbers[2]].myPos.x + (koeficients[1] * (yPos - points[indexesNumbers[2]].myPos.y)));
                }

                if (leftRight) {
                    minX = xPos1;
                    maxX = xPos2;
                }
                else {
                    minX = xPos2;
                    maxX = xPos1;
                }

                if (minX <= 0) {
                    minX = 0;
                }

                if (maxX >= cameraInfo.screenWidth) {
                    maxX = cameraInfo.screenWidth-1;
                }

                if (minX >= cameraInfo.screenWidth) {
                    minX = cameraInfo.screenWidth-2;
                }

                if (maxX <= 0) {
                    maxX = 0;
                }

                for (int xPos = minX; xPos < maxX+1; xPos += 1) {
                    uint8_t globalZ = (uint8_t) points[2].myPos.z + (xPos - points[2].myPos.x) * gradiant[0] + (yPos - points[2].myPos.y) * gradiant[1];

                    if (blockification) {
                        for (int yPosReal = yPos; yPosReal < yPos + blockyfacion; yPosReal += 1) {
                            if (yPosReal >= cameraInfo.screenHeight) {
                                yPosReal += blockyfacion;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] ) {
                                if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                    colorsBuffer[xPos + (yPosReal * cameraInfo.screenWidth)] = convertedOutLine;
                                }
                                else {
                                    colorsBuffer[xPos + (yPosReal * cameraInfo.screenWidth)] = convertedColor;
                                }
                                zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }

                    else {

                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] ) {
                            if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = convertedOutLine;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = convertedColor;
                            }
                            zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }

        else {
            bool leftRight = false;
            int minY = 0;
            int maxY = 0;

            if (points[indexesNumbers[0]].myPos.y + (koeficients[2] * (minsAmaxs[2] - points[indexesNumbers[0]].myPos.x)) < points[indexesNumbers[0]].myPos.y + (koeficients[0] * (minsAmaxs[2] - points[indexesNumbers[0]].myPos.x))) {
                leftRight = true;
            }

            for (int xPos = minsAmaxs[2]; xPos < minsAmaxs[0]+1; xPos += blockyfacion) {

                int yPos2 = std::round(points[indexesNumbers[0]].myPos.y + (koeficients[0] * (xPos - points[indexesNumbers[0]].myPos.x)));
                int yPos1 = 0;

                if (xPos >= points[indexesNumbers[2]].myPos.x) {
                    yPos1 = std::round(points[indexesNumbers[0]].myPos.y + (koeficients[2] * (xPos - points[indexesNumbers[0]].myPos.x)));
                }

                else {
                    yPos1 = std::round(points[indexesNumbers[2]].myPos.y + (koeficients[1] * (xPos - points[indexesNumbers[2]].myPos.x)));
                }

                if (leftRight) {
                    minY = yPos1;
                    maxY = yPos2;
                }
                else {
                    minY = yPos2;
                    maxY = yPos1;
                }

                if (minY <= 0) {
                    minY = 0;
                }

                if (maxY >= cameraInfo.screenHeight) {
                    maxY = cameraInfo.screenHeight-2;
                }

                if (minY >= cameraInfo.screenHeight) {
                    minY = cameraInfo.screenHeight-2;
                }

                if (maxY <= 0) {
                    maxY = 0;
                }

                for (int yPos = minY; yPos < maxY+1; yPos += 1) {
                    uint8_t globalZ = (uint8_t) points[2].myPos.z + (xPos - points[2].myPos.x) * gradiant[0] + (yPos - points[2].myPos.y) * gradiant[1];

                    if (blockification) {
                        for (int xPosReal = xPos; xPosReal < xPos + blockyfacion; xPosReal += 1) {
                            if (xPosReal >= cameraInfo.screenWidth) {
                                xPosReal += blockyfacion;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] ) {
                                if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                    colorsBuffer[xPosReal + (yPos * cameraInfo.screenWidth)] = convertedOutLine;
                                }
                                else {
                                    colorsBuffer[xPosReal + (yPos * cameraInfo.screenWidth)] = convertedColor;
                                }
                                zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }
                    else {
                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] ) {
                            if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = convertedOutLine;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = convertedColor;
                            }
                            zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }
    }
};


class Point_float {
private:
    Position3D_float position = {};

public:
    simple3D_Pos_float xVector;
    simple3D_Pos_float zVector;
    simple3D_Pos_float yVector;

    Point_float(simple3D_Pos_float impPosition) {
        this->position.myPos = impPosition;
        this->xVector = {0,0,0};
        this->zVector = {0,0,0};
        this->yVector = {0,0,0};
    }

    void setupUnitVectors(simple3D_Pos_float referencePoint) {
        if (position.myPos.x >= referencePoint.x) {
            xVector = {1,0,0};
        }
        if (position.myPos.x < referencePoint.x) {
            xVector = {-1,0,0};
        }
        
        if (position.myPos.y >= referencePoint.y) {
            yVector = {0,1,0};
        }
        if (position.myPos.y < referencePoint.y) {
            yVector = {0,-1,0};
        }
        
        if (position.myPos.z >= referencePoint.z) {
            zVector = {0,0,1};
        }
        if (position.myPos.z < referencePoint.z) {
            zVector = {0,0,-1};
        }
    }

    Position3D_float getPos() {
        return position;
    }

    void setPos(simple3D_Pos_float changePos) {
        position.myPos = changePos;
    }

    void changePos(simple3D_Pos_float changePos) {
        position.myPos.x += changePos.x;
        position.myPos.y += changePos.y;
        position.myPos.z += changePos.z;
    }

    Position3D_float getRelativePos(Vector3D_float &headingVec, Vector3D_float &upVector, Vector3D_float &rightVector, Position3D_float &headingOrigin) {

        Vector3D_float vectorOS = headingOrigin.makeAVector(position);

        return Position3D_float(simple3D_Pos_float(vectorOS.dotProduct(rightVector), vectorOS.dotProduct(upVector), vectorOS.dotProduct(headingVec)));
    }
};


enum class ObjectTypes {
    cube = 1,
    unique = 2
};

bool AABBCCColision(simple3D_Pos_float objB, simple3D_Pos_float &sizeA) {
    if (- sizeA.x < objB.x && sizeA.x > objB.x) {
        if (- sizeA.y < objB.y && sizeA.y > objB.y) {
            if (- sizeA.z < objB.z && sizeA.z > objB.z) {
                return true;
            }
        }
    }
    return false;
}


simple3D_Pos_float locilazePoint(Position3D_float &playerPos, Position3D_float &centrePoint, std::array<Vector3D_float, 3> &unitVectorsCentre) {
    Vector3D_float centPointVec = centrePoint.makeAVector(playerPos);
    return simple3D_Pos_float(unitVectorsCentre[0].dotProduct(centPointVec), unitVectorsCentre[1].dotProduct(centPointVec), unitVectorsCentre[2].dotProduct(centPointVec));
}


bool boundBoxCol(Position3D_float &playerPos, Position3D_float &centrePoint, std::array<Vector3D_float, 3> &unitVectorsCentre, simple3D_Pos_float &playerSize, simple3D_Pos_float &blockSize) {
    std::array<Position3D_float, 9> points = {
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, -playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, -playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, -playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, -playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(0, 0, 0)))
    };

    for (int i = 0; i < 9; i += 1) {
        if (AABBCCColision(locilazePoint(points[i], centrePoint, unitVectorsCentre), blockSize)) {
            return true;
        }
    }
    return false;
}


float findSmallestThree(std::array<float, 3> things) {
    float returningThing = things[0];
    for (int i = 0; i < 3; i += 1) {
        if (things[i] <= returningThing) {
            returningThing = things[i];
        }
    }
    return returningThing;
}


enum class InputTypes {
    forward = 1,
    back = 2,
    left = 3,
    right = 4,
    up = 5,
    down = 6,
    cameraLeft = 7,
    cameraRight = 8,
    cameraUp = 9,
    cameraDown = 10
};


struct playerHelpfulVals {
    float sinAngleY, cosAngleY, cosAngleZ, sinAngleZ;
    float angleY, angleZ;

    playerHelpfulVals(float angleY = 0, float angleZ = 0) {
        this->angleY = angleY;
        this->angleZ = angleZ;
        this->sinAngleY = std::sin(angleY);
        this->cosAngleY = std::cos(angleY);
        this->sinAngleZ = std::sin(angleZ);
        this->cosAngleZ = std::cos(angleZ);
    }

    void changeAngleY(float changeY) {
        angleY += changeY;
        sinAngleY = std::sin(angleY);
        cosAngleY = std::cos(angleY);
    }

    void changeAngleZ(float changeZ) {
        angleZ += changeZ;
        sinAngleZ = std::sin(angleZ);
        cosAngleZ = std::cos(angleZ);
    }
};



class Object3D_float {
private:
    // links in style of having xxx and then another xxx ...
    // for now only cube collisions work

    std::vector<int> links;
    std::vector<Point_float> points;
    simple3D_Pos_float centrePoint;
    SimpleColor objectColor;
    int numsOfPoints;
    int numsOfFaces;
    bool stroked;
    SimpleColor outlineCol;
    simple3D_Pos_float objectSize;
    float outLineSize;
    float outerRad;
    float innerRad;
    std::array<Vector3D_float, 3> centreVec = {Vector3D_float(simple3D_Pos_float(1,0,0)), Vector3D_float(simple3D_Pos_float(0,1,0)), Vector3D_float(simple3D_Pos_float(0,0,1))};

public:
    bool visibility;
    bool colision;

    Object3D_float(int numOfPoints, int numFaces, std::vector<Point_float> &points, simple3D_Pos_float &centre,
        std::vector<int> impLinks, simple3D_Pos_float impSize = simple3D_Pos_float(0,0,0), SimpleColor objectColor = SimpleColor(0,0,0,255),
        SimpleColor outlineCol = SimpleColor(0,0,0,255), bool stroked = false, float outLineSize = 1, bool visibility = false, bool colision = false) {
        this->outLineSize = outLineSize;
        this->stroked = stroked;
        this->outlineCol = outlineCol;
        this->objectColor = objectColor;
        this->numsOfFaces = numFaces;
        this->numsOfPoints = numOfPoints;
        this->centrePoint = centre;
        this->objectSize.x = impSize.x/2;
        this->objectSize.y = impSize.y/2;
        this->objectSize.z = impSize.z/2;
        this->links = impLinks;
        this->visibility = visibility;
        this->colision = colision;
        this->outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        this->innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
        this->centreVec[0] = Vector3D_float(simple3D_Pos_float(1,0,0));
        this->centreVec[1] = Vector3D_float(simple3D_Pos_float(0,1,0));
        this->centreVec[2] = Vector3D_float(simple3D_Pos_float(0,0,1));

        for (int i = 0; i < numOfPoints; i += 1) {
            this->points.push_back(points[i]);
            this->points[i].setupUnitVectors(centrePoint);
        }
    }

    bool colide(Position3D_float &playerPos, simple3D_Pos_float &playerSize, float playerRad) {
        Position3D_float centrePosition3D = Position3D_float(centrePoint);
        float distance = std::pow(playerPos.myPos.x - centrePoint.x, 2) + std::pow(playerPos.myPos.y - centrePoint.y, 2) + std::pow(playerPos.myPos.z - centrePoint.z, 2);
        if (distance <= std::pow(playerRad + outerRad,2)) {
            if (distance <= std::pow(playerRad + innerRad,2)) {
                return true;
            }
            if (boundBoxCol(playerPos, centrePosition3D, centreVec, playerSize, objectSize)) {
                return true;
            }
        }
        return false;
    }

    void updatePoses(int i) {
        points[i].setPos(simple3D_Pos_float(
                centrePoint.x + (points[i].xVector.x * objectSize.x) + (points[i].yVector.x * objectSize.y) + (points[i].zVector.x * objectSize.z),
                centrePoint.y + (points[i].xVector.y * objectSize.x) + (points[i].yVector.y * objectSize.y) + (points[i].zVector.y * objectSize.z),
                centrePoint.z + (points[i].xVector.z * objectSize.x) + (points[i].yVector.z * objectSize.y) + (points[i].zVector.z * objectSize.z)));
    }

    void rotates(float angleYZ, float angleXZ, float angleXY) {
        float cosAngleYZ = std::cos(angleYZ);
        float sinAngleYZ = std::sin(angleYZ);
        float cosAngleXY = std::cos(angleXY);
        float sinAngleXY = std::sin(angleXY);
        float cosAngleXZ = std::cos(angleXZ);
        float sinAngleXZ = std::sin(angleXZ);
        centreVec[0] = Vector3D_float(simple3D_Pos_float(
                (centreVec[0].myPos.x * cosAngleXY - centreVec[0].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[0].myPos.z * sinAngleXZ,
                (centreVec[0].myPos.y * cosAngleXY + centreVec[0].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[0].myPos.z * sinAngleYZ,
                (centreVec[0].myPos.z * cosAngleXZ + centreVec[0].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[0].myPos.y * sinAngleYZ));

        centreVec[1] = Vector3D_float(simple3D_Pos_float(
                (centreVec[1].myPos.x * cosAngleXY - centreVec[1].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[1].myPos.z * sinAngleXZ,
                (centreVec[1].myPos.y * cosAngleXY + centreVec[1].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[1].myPos.z * sinAngleYZ,
                (centreVec[1].myPos.z * cosAngleXZ + centreVec[1].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[1].myPos.y * sinAngleYZ));

        centreVec[2] = Vector3D_float(simple3D_Pos_float(
                (centreVec[2].myPos.x * cosAngleXY - centreVec[2].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[2].myPos.z * sinAngleXZ,
                (centreVec[2].myPos.y * cosAngleXY + centreVec[2].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[2].myPos.z * sinAngleYZ,
                (centreVec[2].myPos.z * cosAngleXZ + centreVec[2].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[2].myPos.y * sinAngleYZ));

        for (int i = 0; i < numsOfPoints; i += 1) {
            // 1. XY rotation, 2. XZ rotation, 3. YZ rotation

            points[i].xVector = simple3D_Pos_float(
                (points[i].xVector.x * cosAngleXY - points[i].xVector.y * sinAngleXY) * cosAngleXZ - points[i].xVector.z * sinAngleXZ,
                (points[i].xVector.y * cosAngleXY + points[i].xVector.x * sinAngleXY) * cosAngleYZ - points[i].xVector.z * sinAngleYZ,
                (points[i].xVector.z * cosAngleXZ + points[i].xVector.x * sinAngleXZ) * cosAngleYZ + points[i].xVector.y * sinAngleYZ);
            
            points[i].yVector = simple3D_Pos_float(
                (points[i].yVector.x * cosAngleXY - points[i].yVector.y * sinAngleXY) * cosAngleXZ - points[i].yVector.z * sinAngleXZ,
                (points[i].yVector.y * cosAngleXY + points[i].yVector.x * sinAngleXY) * cosAngleYZ - points[i].yVector.z * sinAngleYZ,
                (points[i].yVector.z * cosAngleXZ + points[i].yVector.x * sinAngleXZ) * cosAngleYZ + points[i].yVector.y * sinAngleYZ);
            
            points[i].zVector = simple3D_Pos_float(
                (points[i].zVector.x * cosAngleXY - points[i].zVector.y * sinAngleXY) * cosAngleXZ - points[i].zVector.z * sinAngleXZ,
                (points[i].zVector.y * cosAngleXY + points[i].zVector.x * sinAngleXY) * cosAngleYZ - points[i].zVector.z * sinAngleYZ,
                (points[i].zVector.z * cosAngleXZ + points[i].zVector.x * sinAngleXZ) * cosAngleYZ + points[i].zVector.y * sinAngleYZ);
            updatePoses(i);
            
        }
    }

    simple3D_Pos_float getSize() {
        return objectSize;
    }

    simple3D_Pos_float getPos() {
        return centrePoint;
    }

    void setPos(simple3D_Pos_float changePos) {
        centrePoint = changePos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
    }

    void changePos(simple3D_Pos_float changePos) {
        centrePoint.x += changePos.x;
        centrePoint.y += changePos.y;
        centrePoint.z += changePos.z;

        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
    }

    void setSize(simple3D_Pos_float newSize) {
        objectSize = newSize;
        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
    }

    void changeSize(simple3D_Pos_float newSize) {
        objectSize.x += newSize.x;
        objectSize.y += newSize.y;
        objectSize.z += newSize.z;
        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
    }

    void drawOutSDL2(Vector3D_float &headingVec, Vector3D_float &upVector, Vector3D_float &rightVector, Position3D_float &headingOrigin, screenAndCameraInfo &camera_info, uint8_t* colorsBuffer, uint8_t *zBuffer, bool blockify, float detail) {

        std::vector<Position3D_float> pseudoPos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            pseudoPos.push_back((points[i].getRelativePos(headingVec, upVector, rightVector, headingOrigin)).makeIntoScreensCord(camera_info));
        }

        for (int i = 0; i < numsOfFaces; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0 || pseudoPos[links[i+1]].myPos.z < 0 || pseudoPos[links[i+2]].myPos.z < 0) {
                continue;
            }

            ScreenPolygon_float onePolygon = ScreenPolygon_float(pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]], objectColor);
            onePolygon.drawOutPolygonSDL2Fast(zBuffer, colorsBuffer, camera_info, stroked, outlineCol, outLineSize, blockify, detail);
        }
    }
};

Object3D_float createCubePoints(simple3D_Pos_float onePos, simple3D_Pos_float oneSize, SimpleColor objColor, SimpleColor outColor, bool stroked, float outlineSize) {
    std::vector<Point_float> points;
    points.push_back(Point_float(onePos.changedBy(0, 0, 0)));
    points.push_back(Point_float(onePos.changedBy(oneSize.x, 0, 0)));
    points.push_back(Point_float(onePos.changedBy(oneSize.x, oneSize.y, 0)));
    points.push_back(Point_float(onePos.changedBy(oneSize.x, oneSize.y, oneSize.z)));
    points.push_back(Point_float(onePos.changedBy(0, oneSize.y, oneSize.z)));
    points.push_back(Point_float(onePos.changedBy(0, 0, oneSize.z)));
    points.push_back(Point_float(onePos.changedBy(0, oneSize.y, 0)));
    points.push_back(Point_float(onePos.changedBy(oneSize.x, 0, oneSize.z)));

    simple3D_Pos_float centre = simple3D_Pos_float(onePos.x + (oneSize.x/2), onePos.y + (oneSize.y/2), onePos.z + (oneSize.z/2));

    return Object3D_float(8, 36, points, centre, {
         0,2,1, 0,6,2,  // Z=0
         3,4,5, 3,5,7,  // Z=1
         0,5,4, 0,4,6,  // X=0
         1,2,3, 1,3,7,  // X=1
         0,1,7, 0,7,5,  // Y=0
         2,4,3, 2,6,4   // Y=1
    }, oneSize, objColor, outColor, stroked, outlineSize, true, true);
}


class Player_float {
private:
    screenAndCameraInfo myCameraInfo;
    float speed;
    playerHelpfulVals values;
    playerHelpfulVals valuesUp;
    playerHelpfulVals valuesRight;
    Vector3D_float headingVec = Vector3D_float(simple3D_Pos_float());
    Vector3D_float rightVec = Vector3D_float(simple3D_Pos_float());
    Vector3D_float upVec = Vector3D_float(simple3D_Pos_float());
    Position3D_float myPos = Position3D_float(simple3D_Pos_float());
    bool pixelization;
    float detailLevel;
    float sizeRadius;
    bool coliding = false;
    simple3D_Pos_float lastPos;

public:

    Player_float(float speed = 0.5, int screenHeight = 1080, int screenWidth = 1920, int fov = 90, simple3D_Pos_float beginPos = simple3D_Pos_float(0,0,0), bool pixelization = true, float detaiLevel = 1, float sizeRadius = 10) {
        this->myPos.setPosition(beginPos);
        this->speed = speed;
        this->pixelization = pixelization;
        this->detailLevel = detaiLevel;
        this->sizeRadius = sizeRadius;
        this->coliding = false;
        
        this->values = playerHelpfulVals(0,0);
        this->valuesUp = playerHelpfulVals(this->values.angleY,(this->values.angleZ - M_PI / 2));
        this->valuesRight = playerHelpfulVals((this->values.angleY + M_PI / 2),this->values.angleZ);
        
        this->headingVec.setVector(simple3D_Pos_float(this->values.cosAngleY * this->values.cosAngleZ, this->values.sinAngleY * this->values.cosAngleZ, this->values.sinAngleZ));
        this->rightVec.setVector(simple3D_Pos_float(this->valuesRight.cosAngleY * this->valuesRight.cosAngleZ, this->valuesRight.sinAngleY * this->valuesRight.cosAngleZ, 0));
        this->upVec.setVector(simple3D_Pos_float(this->valuesUp.cosAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleZ));
        
        float radiansFOV = fov * (M_PI / 180);
        this->myCameraInfo = screenAndCameraInfo(((screenWidth / 2) / (std::tan(radiansFOV/2))) * 1.33, ((screenHeight / 2) / (std::tan(radiansFOV/2))), screenHeight, screenWidth);
    }

    void camera(std::vector<Object3D_float> const &objects, uint8_t* colorBuffer, uint8_t* zBuffer) {
        for (Object3D_float oneObject : objects) {
            if (oneObject.visibility) {
                oneObject.drawOutSDL2(headingVec, upVec, rightVec, myPos, myCameraInfo, colorBuffer, zBuffer, pixelization, detailLevel);
            }
        }
    }

    void movement(InputTypes state) {

    if (state == InputTypes::forward) {
        lastPos = myPos.myPos;
        myPos.myPos.x += speed * values.cosAngleY;
        myPos.myPos.y += speed * values.sinAngleY;
    }
    if (state == InputTypes::back) {
        lastPos = myPos.myPos;
        myPos.myPos.x -= speed * values.cosAngleY;
        myPos.myPos.y -= speed * values.sinAngleY;
    }
    if (state == InputTypes::right) {
        lastPos = myPos.myPos;
        myPos.myPos.x += speed * valuesRight.cosAngleY;
        myPos.myPos.y += speed * valuesRight.sinAngleY;
    }
    if (state == InputTypes::left) {
        lastPos = myPos.myPos;
        myPos.myPos.x -= speed * valuesRight.cosAngleY;
        myPos.myPos.y -= speed * valuesRight.sinAngleY;
    }
    if (state == InputTypes::up) {
        lastPos = myPos.myPos;
        myPos.myPos.z += speed;
    }
    if (state == InputTypes::down) {
        lastPos = myPos.myPos;
        myPos.myPos.z -= speed;
    }
    if (state == InputTypes::cameraDown) {
        if (values.angleZ >= -1.4) {
            values.changeAngleZ(-0.05);
            valuesUp.changeAngleZ(-0.05);

            headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
        }
    }
    if (state == InputTypes::up) {
        if (values.angleZ <= 1) {
            values.changeAngleZ(0.05);
            valuesUp.changeAngleZ(0.05);

            headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
        }
    }
    if (state == InputTypes::left) {
        values.changeAngleY(-0.05);
        valuesUp.changeAngleY(-0.05);
        valuesRight.changeAngleY(-0.05);

        headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
        rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
        upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
    }
    if (state == InputTypes::right) {
        values.changeAngleY(0.05);
        valuesUp.changeAngleY(0.05);
        valuesRight.changeAngleY(0.05);

        headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
        rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
        upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
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

    return 0;
}