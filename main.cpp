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
#include <thread>
#include <mutex>

//fully operational 3D raycasting engine

double getRandomDouble(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0f, 1.0f);
    double returnableValue = double((dis(gen)*(max-min+1))+min);
    return returnableValue;
}

int getRandomInt(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
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
public:
    double absoluteLenght;
    simple3D_Pos_Double myPos;

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

    double dotProductSimple(simple3D_Pos_Double &secondVec) {
        return (secondVec.x * myPos.x) + (secondVec.y * myPos.y) + (secondVec.z * myPos.z);
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

    Vector3D_Double crossProduct3D(Vector3D_Double &secondVec) {
        return Vector3D_Double(simple3D_Pos_Double((myPos.y * secondVec.myPos.z) - (myPos.z * secondVec.myPos.y), (myPos.x * secondVec.myPos.z) - (myPos.z * secondVec.myPos.x), (myPos.x * secondVec.myPos.y) - (myPos.y * secondVec.myPos.x)));
    }

    double getDeterminant(Vector3D_Double &normal, Vector3D_Double &headingVec) {
        return normal.dotProduct(headingVec);
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

    void changePosition(simple3D_Pos_Double impPos) {
        myPos.x += impPos.x;
        myPos.y += impPos.y;
        myPos.z += impPos.z;
    }

    Vector3D_Double makeAVector(Position3D_Double &secondPos) {
        return Vector3D_Double(simple3D_Pos_Double(secondPos.myPos.x - myPos.x,secondPos.myPos.y - myPos.y,secondPos.myPos.z - myPos.z));
    }
    
    Vector3D_Double makeAUnitVector(Position3D_Double secondPos) {
        double distance = std::sqrt(std::pow(myPos.x - secondPos.myPos.x, 2) + std::pow(myPos.y - secondPos.myPos.y, 2) + std::pow(myPos.z - secondPos.myPos.z, 2));

        if (distance < 0.05) {
            distance = 0.06;
        }

        return Vector3D_Double(simple3D_Pos_Double((secondPos.myPos.x - myPos.x) / distance,(secondPos.myPos.y - myPos.y) / distance,(secondPos.myPos.z - myPos.z) / distance));
    }

    Vector3D_Double makeA2DVector(Position3D_Double &secondPos) {
        return Vector3D_Double(simple3D_Pos_Double(myPos.x - secondPos.myPos.x,myPos.y - secondPos.myPos.y,0));
    }

    double absoluteDistance(Position3D_Double &secondPos) {
        return std::sqrt(std::pow(secondPos.myPos.x - myPos.x,2) + std::pow(secondPos.myPos.y - myPos.y,2) + std::pow(secondPos.myPos.z - myPos.z,2));
    }

    double absoluteDistanceSimple(simple3D_Pos_Double &secondPos) {
        return std::sqrt(std::pow(secondPos.x - myPos.x,2) + std::pow(secondPos.y - myPos.y,2) + std::pow(secondPos.z - myPos.z,2));
    }

    Position3D_Double changedBy(simple3D_Pos_Double changePos) {
        return Position3D_Double(simple3D_Pos_Double(myPos.x + changePos.x, myPos.y + changePos.y, myPos.z + changePos.z));
    }

    Position3D_Double makeIntoScreensCord(screenAndCameraInfo const &impInfo) {

        if (myPos.z > 0.01) {
            return Position3D_Double(simple3D_Pos_Double(std::round(((myPos.x / myPos.z) * impInfo.numberAmpX) + (impInfo.screenWidth / 2)), std::round(((myPos.y / myPos.z) * impInfo.numberAmpY) + (impInfo.screenHeight / 2)), myPos.z));
        }

        if (std::abs(myPos.z) < 0.005) {
            myPos.z = 0.006;
        }

        return Position3D_Double(simple3D_Pos_Double(std::round(((myPos.x / 0.01) * impInfo.numberAmpX) + (impInfo.screenWidth / 2)), std::round(((myPos.y / 0.01) * impInfo.numberAmpY) + (impInfo.screenHeight / 2)), myPos.z));
    }

    Position3D_Double makeIntoGradiantable(screenAndCameraInfo const &impInfo) {
        if (std::abs(myPos.z) < 0.005) {
            myPos.z = 0.006;
        }
        return Position3D_Double(simple3D_Pos_Double(std::round(((myPos.x / myPos.z) * impInfo.numberAmpX) + (impInfo.screenWidth / 2)), std::round(((myPos.y / myPos.z) * impInfo.numberAmpY) + (impInfo.screenHeight / 2)), 1 / myPos.z));
    }
};


std::array<double, 2> getGradiantsDouble(Position3D_Double &pointA, Position3D_Double &pointB, Position3D_Double &pointC) {
    double diffX1 = pointB.myPos.x - pointA.myPos.x;
    double diffX2 = pointC.myPos.x - pointB.myPos.x;
    double diffY1 = pointB.myPos.y - pointA.myPos.y;
    double diffY2 = pointC.myPos.y - pointB.myPos.y;
    double diffZ1 = pointB.myPos.z - pointA.myPos.z;
    double diffZ2 = pointC.myPos.z - pointB.myPos.z;
    double determinant = diffX1 * diffY2 - diffX2 * diffY1;
    if (std::abs(determinant) <= 10) {
        return {0,0};
    }
    double gradiantX = (diffY2 * diffZ1 - diffZ2 * diffY1) / (determinant);
    double gradiantY = (diffX1 * diffZ2 - diffZ1 * diffX2) / (determinant);
    return {gradiantX, gradiantY};
}


class ScreenPolygon_Double {
private:
    std::array<Position3D_Double, 3> points = {};
    std::array<Position3D_Double, 3> pointsGradiantable = {};
    SimpleColor myColor = {};
    
    std::array<int, 4> localMinsAmaxs;
    std::array<double, 2> localGradiant;
    uint32_t localConvertedColor;
    uint32_t localOutlineColor;
    bool localxMajority = false;
    double localBlockification = 1;
    std::array<double, 3> localKoeficients;
    std::array<int, 3> localIndexesNumbers;

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

    std::array<double, 3> getKoeficients(bool xMajority, std::array<int, 3> const &infos) {
        std::array<double, 3> returningKoeficients = {};

        double diveder0y = (points[infos[0]].myPos.y - points[infos[1]].myPos.y);
        double diveder1y = (points[infos[2]].myPos.y - points[infos[1]].myPos.y);
        double diveder2y = (points[infos[0]].myPos.y - points[infos[2]].myPos.y);

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

        double diveder0x = (points[infos[0]].myPos.x - points[infos[1]].myPos.x);
        double diveder1x = (points[infos[2]].myPos.x - points[infos[1]].myPos.x);
        double diveder2x = (points[infos[0]].myPos.x - points[infos[2]].myPos.x);

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

    ScreenPolygon_Double(Position3D_Double point1 = Position3D_Double(simple3D_Pos_Double(0,0,0)), Position3D_Double point2 = Position3D_Double(simple3D_Pos_Double(0,0,0)), Position3D_Double point3 = Position3D_Double(simple3D_Pos_Double(0,0,0)),
        Position3D_Double point1G = Position3D_Double(simple3D_Pos_Double(0,0,0)), Position3D_Double point2G = Position3D_Double(simple3D_Pos_Double(0,0,0)), Position3D_Double point3G = Position3D_Double(simple3D_Pos_Double(0,0,0)),
        SimpleColor impCol = SimpleColor(0,0,0,0)) {
        this->points[0] = point1;
        this->points[1] = point2;
        this->points[2] = point3;
        this->pointsGradiantable[0] = point1G;
        this->pointsGradiantable[1] = point2G;
        this->pointsGradiantable[2] = point3G;
        this->myColor = impCol;
    }

    void changeColor(SimpleColor const &impCol) {
        myColor.red = impCol.red;
        myColor.green = impCol.green;
        myColor.blue = impCol.blue;
    }
    
    void prepresentAssets(screenAndCameraInfo &cameraInfo, SimpleColor outlineColor, SimpleColor insideColor, bool blockification, double blockDetail) {
        localMinsAmaxs = minsAndMaxs(cameraInfo);
        localGradiant = getGradiantsDouble(pointsGradiantable[0], pointsGradiantable[1], pointsGradiantable[2]);

        localConvertedColor = insideColor.convertToBinary();
        localOutlineColor = outlineColor.convertToBinary();

        localxMajority = compareMinsAndMaxs(localMinsAmaxs);

        localIndexesNumbers = minsAndMaxsAMiddle(localxMajority);

        localKoeficients = getKoeficients(localxMajority, localIndexesNumbers);

        localBlockification = 1;

        if (blockification) {
            double sizer = (localMinsAmaxs[0] - localMinsAmaxs[2]) * (localMinsAmaxs[1] - localMinsAmaxs[3]) / 2;
            double middleZ = (points[0].myPos.z + points[1].myPos.z + points[2].myPos.z) / 3;
            localBlockification = int(blockDetail * sqrt(sizer) / middleZ);
        }

        if (localBlockification <= 1) {
            localBlockification = 1;
        }

        if (localBlockification >= 20) {
            localBlockification = 20;
        }
    }


    void drawOutPolygonSDL2SuperFast(double* zBufferImp, uint32_t* colorsBuffer, int pitch, screenAndCameraInfo const &cameraInfo, bool outLine, int outlineThickness, bool blockification, double blockDetail) {

        if (localxMajority) {
            bool leftRight = false;
            int minX = 0;
            int maxX = 0;
            if (points[localIndexesNumbers[0]].myPos.x + (localKoeficients[0] * (localMinsAmaxs[3] - points[localIndexesNumbers[0]].myPos.y)) > points[localIndexesNumbers[0]].myPos.x + (localKoeficients[2] * (localMinsAmaxs[3] - points[localIndexesNumbers[0]].myPos.y))) {
                leftRight = true;
            }

            for (int yPos = localMinsAmaxs[3]; yPos < localMinsAmaxs[1]+1; yPos += localBlockification) {
                int xPos2 = std::round(points[localIndexesNumbers[0]].myPos.x + (localKoeficients[0] * (yPos - points[localIndexesNumbers[0]].myPos.y)));
                int xPos1 = 0;
                if (yPos >= points[localIndexesNumbers[2]].myPos.y) {
                    xPos1 = std::round(points[localIndexesNumbers[0]].myPos.x + (localKoeficients[2] * (yPos - points[localIndexesNumbers[0]].myPos.y)));
                }

                else {
                    xPos1 = std::round(points[localIndexesNumbers[2]].myPos.x + (localKoeficients[1] * (yPos - points[localIndexesNumbers[2]].myPos.y)));
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
                    minX = cameraInfo.screenWidth-1;
                }

                if (maxX <= 0) {
                    maxX = 0;
                }

                for (int xPos = minX; xPos < maxX+1; xPos += 1) {
                    double globalZ = 1 / (pointsGradiantable[0].myPos.z + (xPos - pointsGradiantable[0].myPos.x) * localGradiant[0] + (yPos - pointsGradiantable[0].myPos.y) * localGradiant[1]);

                    if (blockification) {
                        for (int yPosReal = yPos; yPosReal < yPos + localBlockification; yPosReal += 1) {
                            if (yPosReal >= cameraInfo.screenHeight) {
                                yPosReal += localBlockification;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] && globalZ > 0) {
                                if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                    colorsBuffer[xPos + (yPosReal * pitch)] = localOutlineColor;
                                }
                                else {
                                    colorsBuffer[xPos + (yPosReal * pitch)] = localConvertedColor;
                                }
                                zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }

                    else {

                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                            if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                colorsBuffer[xPos + (yPos * pitch)] = localOutlineColor;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * pitch)] = localConvertedColor;
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

            if (points[localIndexesNumbers[0]].myPos.y + (localKoeficients[2] * (localMinsAmaxs[2] - points[localIndexesNumbers[0]].myPos.x)) < points[localIndexesNumbers[0]].myPos.y + (localKoeficients[0] * (localMinsAmaxs[2] - points[localIndexesNumbers[0]].myPos.x))) {
                leftRight = true;
            }

            for (int xPos = localMinsAmaxs[2]; xPos < localMinsAmaxs[0]+1; xPos += localBlockification) {

                int yPos2 = std::round(points[localIndexesNumbers[0]].myPos.y + (localKoeficients[0] * (xPos - points[localIndexesNumbers[0]].myPos.x)));
                int yPos1 = 0;

                if (xPos >= points[localIndexesNumbers[2]].myPos.x) {
                    yPos1 = std::round(points[localIndexesNumbers[0]].myPos.y + (localKoeficients[2] * (xPos - points[localIndexesNumbers[0]].myPos.x)));
                }

                else {
                    yPos1 = std::round(points[localIndexesNumbers[2]].myPos.y + (localKoeficients[1] * (xPos - points[localIndexesNumbers[2]].myPos.x)));
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
                    maxY = cameraInfo.screenHeight-1;
                }

                if (minY >= cameraInfo.screenHeight) {
                    minY = cameraInfo.screenHeight-1;
                }

                if (maxY <= 0) {
                    maxY = 0;
                }

                for (int yPos = minY; yPos < maxY+1; yPos += 1) {
                    double globalZ = 1 / (pointsGradiantable[0].myPos.z + (xPos - pointsGradiantable[0].myPos.x) * localGradiant[0] + (yPos - pointsGradiantable[0].myPos.y) * localGradiant[1]);

                    if (blockification) {
                        for (int xPosReal = xPos; xPosReal < xPos + localBlockification; xPosReal += 1) {
                            if (xPosReal >= cameraInfo.screenWidth) {
                                xPosReal += localBlockification;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                                if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                    colorsBuffer[xPosReal + (yPos * pitch)] = localOutlineColor;
                                }
                                else {
                                    colorsBuffer[xPosReal + (yPos * pitch)] = localConvertedColor;
                                }
                                zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }
                    else {
                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                            if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                colorsBuffer[xPos + (yPos * pitch)] = localOutlineColor;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * pitch)] = localConvertedColor;
                            }
                            zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }
    }

    void drawOutPolygonSDL2Fast(double* zBufferImp, uint32_t* colorsBuffer, int pitch, screenAndCameraInfo const &cameraInfo, bool outLine, SimpleColor outLineCol, int outlineThickness, bool blockification, double blockDetail) {
        std::array<int, 4> minsAmaxs = minsAndMaxs(cameraInfo);
        std::array<double, 2> gradiant = getGradiantsDouble(pointsGradiantable[0], pointsGradiantable[1], pointsGradiantable[2]);

        uint32_t convertedColor = myColor.convertToBinary();
        uint32_t convertedOutLine = outLineCol.convertToBinary();

        bool xMajority = compareMinsAndMaxs(minsAmaxs);

        std::array<int, 3> indexesNumbers = minsAndMaxsAMiddle(xMajority);

        std::array<double, 3> koeficients = getKoeficients(xMajority, indexesNumbers);

        double blockyfacion = 1;

        if (blockification) {
            double sizer = (minsAmaxs[0] - minsAmaxs[2]) * (minsAmaxs[1] - minsAmaxs[3]) / 2;
            double middleZ = (points[0].myPos.z + points[1].myPos.z + points[2].myPos.z) / 3;
            blockyfacion = int(blockDetail * sqrt(sizer) / middleZ);
        }

        if (blockyfacion <= 1) {
            blockyfacion = 1;
        }

        if (blockyfacion >= 20) {
            blockyfacion = 20;
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
                    minX = cameraInfo.screenWidth-1;
                }

                if (maxX <= 0) {
                    maxX = 0;
                }

                for (int xPos = minX; xPos < maxX+1; xPos += 1) {
                    double globalZ = 1 / (pointsGradiantable[0].myPos.z + (xPos - pointsGradiantable[0].myPos.x) * gradiant[0] + (yPos - pointsGradiantable[0].myPos.y) * gradiant[1]);

                    if (blockification) {
                        for (int yPosReal = yPos; yPosReal < yPos + blockyfacion; yPosReal += 1) {
                            if (yPosReal >= cameraInfo.screenHeight) {
                                yPosReal += blockyfacion;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] && globalZ > 0) {
                                if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                    colorsBuffer[xPos + (yPosReal * pitch)] = convertedOutLine;
                                }
                                else {
                                    colorsBuffer[xPos + (yPosReal * pitch)] = convertedColor;
                                }
                                zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }

                    else {

                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                            if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                colorsBuffer[xPos + (yPos * pitch)] = convertedOutLine;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * pitch)] = convertedColor;
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
                    maxY = cameraInfo.screenHeight-1;
                }

                if (minY >= cameraInfo.screenHeight) {
                    minY = cameraInfo.screenHeight-1;
                }

                if (maxY <= 0) {
                    maxY = 0;
                }

                for (int yPos = minY; yPos < maxY+1; yPos += 1) {
                    double globalZ = 1 / (pointsGradiantable[0].myPos.z + (xPos - pointsGradiantable[0].myPos.x) * gradiant[0] + (yPos - pointsGradiantable[0].myPos.y) * gradiant[1]);

                    if (blockification) {
                        for (int xPosReal = xPos; xPosReal < xPos + blockyfacion; xPosReal += 1) {
                            if (xPosReal >= cameraInfo.screenWidth) {
                                xPosReal += blockyfacion;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                                if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                    colorsBuffer[xPosReal + (yPos * pitch)] = convertedOutLine;
                                }
                                else {
                                    colorsBuffer[xPosReal + (yPos * pitch)] = convertedColor;
                                }
                                zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }
                    else {
                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                            if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                colorsBuffer[xPos + (yPos * pitch)] = convertedOutLine;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * pitch)] = convertedColor;
                            }
                            zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }
    }

    void drawOutPolygonQuickSDL2(double* zBufferImp, uint32_t* colorsBuffer, int pitch, screenAndCameraInfo const &cameraInfo, bool outLine, SimpleColor outLineCol, int outlineThickness, bool blockification, double blockDetail) {

        std::array<int, 4> minsAmaxs = minsAndMaxs(cameraInfo);
        std::array<double, 2> gradiant = getGradiantsDouble(pointsGradiantable[0], pointsGradiantable[1], pointsGradiantable[2]);

        uint32_t convertedColor = myColor.convertToBinary();
        uint32_t convetedOutLine = outLineCol.convertToBinary();

        double sizer = (minsAmaxs[0] - minsAmaxs[2]) * (minsAmaxs[1] - minsAmaxs[3])/2;
        double outLineBoundary = outlineThickness * sqrt(sizer);

        int blockyfacion = 1;

        if (blockification) {
            double middleZ = (points[0].myPos.z + points[1].myPos.z + points[2].myPos.z) / 3;

            blockyfacion = int(blockDetail * sqrt(sizer) / middleZ);

            if (blockyfacion <= 0.5) {
                blockyfacion = 1;
            }
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
                            colorsBuffer[xPos2 + (yPos2 * pitch)] = pixelColor;
                            zBufferImp[xPos2 + (yPos2 * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }
    }
};


class Point_Double {
private:
    Position3D_Double position = {};

public:
    simple3D_Pos_Double xVector;
    simple3D_Pos_Double zVector;
    simple3D_Pos_Double yVector;

    Point_Double(simple3D_Pos_Double impPosition) {
        this->position.myPos = impPosition;
        this->xVector = {0,0,0};
        this->zVector = {0,0,0};
        this->yVector = {0,0,0};
    }

    void setupUnitVectors(simple3D_Pos_Double referencePoint) {
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

    Position3D_Double getPos() {
        return position;
    }

    Position3D_Double& getPosRef() {
        return position;
    }

    void setPos(simple3D_Pos_Double changePos) {
        position.myPos = changePos;
    }

    void changePos(simple3D_Pos_Double changePos) {
        position.myPos.x += changePos.x;
        position.myPos.y += changePos.y;
        position.myPos.z += changePos.z;
    }

    Position3D_Double getRelativePos(Vector3D_Double &headingVec, Vector3D_Double &upVector, Vector3D_Double &rightVector, Position3D_Double &headingOrigin) {

        Vector3D_Double vectorOS = headingOrigin.makeAVector(position);

        return Position3D_Double(simple3D_Pos_Double(vectorOS.dotProduct(rightVector), vectorOS.dotProduct(upVector), vectorOS.dotProduct(headingVec)));
    }
};


struct LightRay_Double {
    Position3D_Double myPos;
    double intenzity;
    double lenght;
    SimpleColor color;
    double highestLight;
    Vector3D_Double headingVec = Vector3D_Double(simple3D_Pos_Double());
    int maxIndexCur = -1;

    LightRay_Double(Position3D_Double impPos, double intenzity, SimpleColor myCol, Vector3D_Double definingVec, double lenghtConst, double maxDistance) {
        this->myPos = impPos;
        this->intenzity = intenzity;
        this->color = myCol;
        this->highestLight = maxDistance;
        this->headingVec = definingVec;
        this->lenght = lenghtConst;
        this->maxIndexCur = -1;
    }
};

enum class LightTypes {
    pointLike = 1,
    paralel = 2
};


enum class DrawOutModes {
    uint8_tMode = 1,
    sdl2Mode = 2
};


class GlobalPolygon_Double {
private:
    std::array<Point_Double, 3> definingGlobalPoints = {Point_Double(simple3D_Pos_Double()), Point_Double(simple3D_Pos_Double()), Point_Double(simple3D_Pos_Double())};
    std::array<Position3D_Double, 3> definingLocalPoints = {Position3D_Double(), Position3D_Double(), Position3D_Double()};
    std::array<Position3D_Double, 3> definingGradiantablePoints = {Position3D_Double(), Position3D_Double(), Position3D_Double()};
    SimpleColor originalColor;
    SimpleColor outlineColor;
    SimpleColor dislayedColor;
    bool blockification;
    double blockLOD;
    bool shouldDraw;
    std::array<Vector3D_Double, 2> lightingVectors = {Vector3D_Double(simple3D_Pos_Double()), Vector3D_Double(simple3D_Pos_Double())};
    Vector3D_Double mainNormal = Vector3D_Double(simple3D_Pos_Double());
    ScreenPolygon_Double myScreenVer;
    int id;

    void changeColor(SimpleColor &newColor) {
        if (newColor.red >= 255) {
            newColor.red = 255;
        }
        if (newColor.red <= 0) {
            newColor.red = 0;
        }

        if (newColor.green >= 255) {
            newColor.green = 255;
        }
        if (newColor.green <= 0) {
            newColor.green = 0;
        }

        if (newColor.blue >= 255) {
            newColor.blue = 255;
        }
        if (newColor.blue <= 0) {
            newColor.blue = 0;
        }
        dislayedColor = newColor;
    }

    void myScreenVerRetype(screenAndCameraInfo &cameraInfo, bool blockification, double blockDetail) {
        myScreenVer = ScreenPolygon_Double(definingLocalPoints[0], definingLocalPoints[1], definingLocalPoints[2], definingGradiantablePoints[0], definingGradiantablePoints[1], definingGradiantablePoints[2]);
        myScreenVer.prepresentAssets(cameraInfo, outlineColor, dislayedColor, blockification, blockDetail);
    }

public:

    GlobalPolygon_Double(std::array<Point_Double, 3> definingGlobalPoints, std::array<Position3D_Double, 3> definingLocalPoints, std::array<Position3D_Double, 3> definingGradiantablePoints, SimpleColor impCol, int index, bool drawOut) {
        this->definingGlobalPoints = definingGlobalPoints;
        this->definingLocalPoints = definingLocalPoints;
        this->definingGradiantablePoints = definingGradiantablePoints;
        this->originalColor = impCol;
        this->lightingVectors[0] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[1].getPosRef());
        this->lightingVectors[1] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[2].getPosRef());
        this->mainNormal = lightingVectors[0].crossProduct3D(lightingVectors[1]);
        this->dislayedColor = impCol;
        this->myScreenVer = ScreenPolygon_Double(definingLocalPoints[0], definingLocalPoints[1], definingLocalPoints[2], definingGradiantablePoints[0], definingGradiantablePoints[1], definingGradiantablePoints[2]);
        this->id = index;
        this->shouldDraw = drawOut;
    }

    void drawOutSuperFast(double* zBufferImp, uint32_t* colorsBuffer, int pitch, screenAndCameraInfo const &cameraInfo, bool outLine, int outlineThickness, bool blockification, double blockDetail) {
        myScreenVer.drawOutPolygonSDL2SuperFast(zBufferImp, colorsBuffer, pitch, cameraInfo, outLine, outlineThickness, blockification, blockDetail);
    }

    void thisChange(std::array<Point_Double, 3> newdefiningGlobalPoints, std::array<Position3D_Double, 3> newdefiningLocalPoints, std::array<Position3D_Double, 3> newdefiningGradiantablePoints,
        screenAndCameraInfo &cameraInfo, bool blockification, double blockDetail, bool drawOut) {
        definingGlobalPoints = newdefiningGlobalPoints;
        definingLocalPoints = newdefiningLocalPoints;
        definingGradiantablePoints = newdefiningGradiantablePoints;
        lightingVectors[0] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[1].getPosRef());
        lightingVectors[1] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[2].getPosRef());
        mainNormal = lightingVectors[0].crossProduct3D(lightingVectors[1]);
        shouldDraw = drawOut;
        myScreenVerRetype(cameraInfo, blockification, blockDetail);
    }

    void playerChange(std::array<Position3D_Double, 3> newdefiningLocalPoints, std::array<Position3D_Double, 3> newdefiningGradiantablePoints, screenAndCameraInfo &cameraInfo, bool blockification, double blockDetail, bool drawOut) {
        definingLocalPoints = newdefiningLocalPoints;
        definingGradiantablePoints = newdefiningGradiantablePoints;
        shouldDraw = drawOut;
        myScreenVerRetype(cameraInfo, blockification, blockDetail);
    }

    void changeOriginalColor(SimpleColor newColor) {
        if (newColor.red >= 255) {
            newColor.red = 255;
        }
        if (newColor.red <= 0) {
            newColor.red = 0;
        }

        if (newColor.green >= 255) {
            newColor.green = 255;
        }
        if (newColor.green <= 0) {
            newColor.green = 0;
        }

        if (newColor.blue >= 255) {
            newColor.blue = 255;
        }
        if (newColor.blue <= 0) {
            newColor.blue = 0;
        }
        originalColor = newColor;
    }

    void rollBackColor() {
        dislayedColor = originalColor;
    }

    void changeLightColor(LightRay_Double &light, double furtheness) {
        double newIntenzity = light.intenzity / (furtheness * light.lenght);
        SimpleColor newColor;
        newColor.red = int(dislayedColor.red + std::round(newIntenzity * light.color.red));
        newColor.green = int(dislayedColor.green + std::round(newIntenzity * light.color.green));
        newColor.blue = int(dislayedColor.blue + std::round(newIntenzity * light.color.blue));
        changeColor(newColor);
    }

    void lightingDuty(LightRay_Double &lightPoint, int curentPos) {
        Vector3D_Double pointVec = definingGlobalPoints[0].getPosRef().makeAVector(lightPoint.myPos);
        double mainDeterminant = mainNormal.dotProduct(lightPoint.headingVec);
        Vector3D_Double secondaryNormal = pointVec.crossProduct3D(lightPoint.headingVec);
        if (std::abs(mainDeterminant) < 0.1) {
            return;
        }
        double countingNum = 1.0 / mainDeterminant;
        double tV = mainNormal.dotProduct(pointVec) * countingNum;
        if (tV > 0.01) {
            double determinantP1 = secondaryNormal.dotProduct(lightingVectors[1]) * countingNum;
            if (determinantP1 > 0.01) {
                double determinantP2 = secondaryNormal.dotProduct(lightingVectors[0]) * countingNum;
                if (determinantP2 > 0.01 && (determinantP1 + determinantP2) <= 1) {
                    if (tV < lightPoint.highestLight) {
                        lightPoint.highestLight = tV;
                        lightPoint.maxIndexCur = curentPos;
                    }
                }
            }
        }
    }

    double lightingRetLenght(LightRay_Double &lightPoint, double renderDistance) {
        Vector3D_Double pointVec = definingGlobalPoints[0].getPosRef().makeAVector(lightPoint.myPos);
        double mainDeterminant = mainNormal.dotProduct(lightPoint.headingVec);
        Vector3D_Double secondaryNormal = pointVec.crossProduct3D(lightPoint.headingVec);
        if (std::abs(mainDeterminant) < 0.1) {
            return renderDistance;
        }
        double countingNum = 1.0 / mainDeterminant;
        double tV = mainNormal.dotProduct(pointVec) * countingNum;
        if (tV > 0.01) {
            double determinantP1 = secondaryNormal.dotProduct(lightingVectors[1]) * countingNum;
            if (determinantP1 > 0.01) {
                double determinantP2 = secondaryNormal.dotProduct(lightingVectors[0]) * countingNum;
                if (determinantP2 > 0.01 && (determinantP1 + determinantP2) <= 1) {
                    return tV;
                }
            }
        }
        return renderDistance;
    }
};


class LightSource {
private:
    LightTypes myType;
    std::vector<LightRay_Double> lightSources;
    Position3D_Double myPos;
    SimpleColor myColor;
    int rayNumber, objectNum;
    double intezity, lenghtDecay;
    double sourceHeight, sourceWidth;
    bool representativeBlock;
    int blockIdMin, blockIdMax;

    void spehereVec(double intezity, double lenghtDecay) {
        double fibonnaciAngle = M_PI * (3 - sqrt(5));
        double quickNum = 1.0 / (rayNumber - 1);
        for (int i = 0; i < rayNumber; i += 1) {
            double yPos = 1 - (2 * i * quickNum);
            double theta = i * fibonnaciAngle;
            double radius = sqrt(1.0 - yPos * yPos);

            lightSources[i] = LightRay_Double(myPos, intezity, myColor, Vector3D_Double(simple3D_Pos_Double(cos(theta) * radius, yPos, sin(theta) * radius)), lenghtDecay, (lenghtDecay * 500) * intezity);
        }
    }

    void polygonVec(double intezity, double lenghtDecay, double sourceHeight, double sourceWidth, Vector3D_Double directionRight, Vector3D_Double directionUp) {
        int numberA = std::round(std::sqrt((sourceHeight * rayNumber) / sourceWidth));
        int numberB = std::round(std::sqrt((sourceWidth * rayNumber) / sourceHeight));
        if (!((numberA * numberB) == rayNumber)) {
            if (numberA > numberB) {
                numberA = rayNumber - numberB;
            }
            else {
                numberB = rayNumber - numberA;
            }
        }
        double diffX = sourceHeight / numberA;
        double diffY = sourceWidth / numberB;
        Vector3D_Double normal = directionRight.crossProduct3D(directionUp);

        for (int i = 0; i < numberB; i += 1) {
            double quickDiff = i * diffY;
            simple3D_Pos_Double heightPos = simple3D_Pos_Double(quickDiff * directionUp.myPos.x, quickDiff * directionUp.myPos.y, quickDiff * directionUp.myPos.z);
            for (int j = 0; j < numberA; j += 1) {
                double quickDiff2 = j * diffX;
                simple3D_Pos_Double rightPos = simple3D_Pos_Double(quickDiff2 * directionRight.myPos.x, quickDiff2 * directionRight.myPos.y, quickDiff2 * directionRight.myPos.z);
                Position3D_Double onePos = Position3D_Double(myPos.changedBy(simple3D_Pos_Double(heightPos.x + rightPos.x, heightPos.y + rightPos.y, heightPos.z + rightPos.z)));
                lightSources[(i * numberB) + j] = LightRay_Double(onePos, intezity, myColor, normal, lenghtDecay, (lenghtDecay * 500) * intezity);
            }
        }
    }

    void lightingCycle( std::vector<GlobalPolygon_Double> &allPolygons) {
        for (int i = 0; i < allPolygons.size(); i += 1) {
            allPolygons[i].rollBackColor();
            for (int j = 0; j < rayNumber; j += 1) {
                if (representativeBlock) {
                    if (i <= blockIdMin && i > blockIdMax) {
                        allPolygons[i].lightingDuty(lightSources[j], i);
                    }
                }
                else {
                    allPolygons[i].lightingDuty(lightSources[j], i);
                }
            }
        }
        for (int i = 0; i < rayNumber; i += 1) {
            LightRay_Double oneRay = lightSources[i];
            int oneIndex = oneRay.maxIndexCur;
            if (!(oneIndex == -1)) {
                allPolygons[oneIndex].changeLightColor(oneRay, oneRay.highestLight);
            }
        }
    }

public:

    LightSource(LightTypes lightType, Position3D_Double impPos, int rayNumber, SimpleColor impCol,
        std::vector<GlobalPolygon_Double> &allPolygons, double intezity, double lenghtDecay, int curPolygon, bool cubeAround = true,
        Vector3D_Double directionRight = Vector3D_Double(simple3D_Pos_Double(0,0,0)), Vector3D_Double directionUp = Vector3D_Double(simple3D_Pos_Double(0,0,0)), double sourceHeight = 0, double sourceWidth = 0) {
        this->myType = lightType;
        this->myPos = impPos;
        this->rayNumber = rayNumber;
        for (int i = 0; i < rayNumber; i += 1) {
            this->lightSources.push_back(LightRay_Double(impPos, 0, myColor, Vector3D_Double(simple3D_Pos_Double(0,0,0)), 0, 0));
        }
        this->myColor = impCol;
        this->intezity = intezity;
        this->lenghtDecay = lenghtDecay;
        this->sourceHeight = sourceHeight;
        this->sourceWidth = sourceWidth;
        this->representativeBlock = cubeAround;
        this->blockIdMin = curPolygon;
        this->blockIdMax = curPolygon + 12;

        this->objectNum = allPolygons.size();
        emitLight(allPolygons, directionRight, directionUp);
    }

    void emitLight(std::vector<GlobalPolygon_Double> &allPolygons, Vector3D_Double directionRight = Vector3D_Double(simple3D_Pos_Double(0,0,0)), Vector3D_Double directionUp = Vector3D_Double(simple3D_Pos_Double(0,0,0))) {
        if (myType == LightTypes::pointLike) {
            spehereVec(intezity, lenghtDecay);
            lightingCycle(allPolygons);
        }
        if (myType == LightTypes::paralel) {
            polygonVec(intezity, lenghtDecay, sourceHeight, sourceWidth, directionRight, directionUp);
            lightingCycle(allPolygons);
        }
    }

    void changePos(simple3D_Pos_Double newPos, std::vector<GlobalPolygon_Double> &allPolygons, Vector3D_Double directionRight = Vector3D_Double(simple3D_Pos_Double(0,0,0)), Vector3D_Double directionUp = Vector3D_Double(simple3D_Pos_Double(0,0,0))) {
        myPos.myPos = newPos;
        emitLight(allPolygons, directionRight, directionUp);
    }
    
    void oneBlockUpdate(std::vector<GlobalPolygon_Double> &allPolygons, int polygonIndex) {
        for (int j = 0; j < rayNumber; j += 1) {
            LightRay_Double& oneRay = lightSources[j];
            double thisRayNum = allPolygons[polygonIndex].lightingRetLenght(oneRay, (oneRay.lenght * 10) * oneRay.intenzity);

            if (thisRayNum < oneRay.highestLight) {
                allPolygons[polygonIndex].changeLightColor(oneRay, thisRayNum);
                if (oneRay.maxIndexCur != -1) {
                    allPolygons[oneRay.maxIndexCur].rollBackColor();
                }
                oneRay.maxIndexCur = polygonIndex;
                oneRay.highestLight = thisRayNum;
            }
        }
    }
};


enum class ObjectTypes {
    cube = 1,
    unique = 2
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


struct playerFullInfo {
    Vector3D_Double headingVector = Vector3D_Double(simple3D_Pos_Double(0,0,0)), rightVector = Vector3D_Double(simple3D_Pos_Double(0,0,0)), upVector = Vector3D_Double(simple3D_Pos_Double(0,0,0));
    screenAndCameraInfo cameraInfo;
    Position3D_Double originPoint;

    playerFullInfo(Vector3D_Double headingVec = Vector3D_Double(simple3D_Pos_Double(0,0,0)), Vector3D_Double upVec = Vector3D_Double(simple3D_Pos_Double(0,0,0)),
        Vector3D_Double rightVec = Vector3D_Double(simple3D_Pos_Double(0,0,0)), Position3D_Double originPoint = Position3D_Double(simple3D_Pos_Double(0,0,0)),
        screenAndCameraInfo cameInfo = screenAndCameraInfo(0,0,0,0)) {
        this->headingVector = headingVec;
        this->rightVector = rightVec;
        this->upVector = upVec;
        this->cameraInfo = cameInfo;
        this->originPoint = originPoint;
    }

    void changeInfo(Vector3D_Double &headingVec, Vector3D_Double &upVec, Vector3D_Double &rightVec, Position3D_Double &neworiginPoint, screenAndCameraInfo &cameInfo) {
        headingVector = headingVec;
        rightVector = rightVec;
        upVector = upVec;
        cameraInfo = cameInfo;
        originPoint = neworiginPoint;
    }
};


struct LODInfo {
    bool blockify;
    double LODLevel;

    LODInfo(bool blocks, double level) {
        this->blockify = blocks;
        this->LODLevel = level;
    }
};


bool AABBCCColision(simple3D_Pos_Double objB, simple3D_Pos_Double &sizeA) {
    if (- sizeA.x < objB.x && sizeA.x > objB.x) {
        if (- sizeA.y < objB.y && sizeA.y > objB.y) {
            if (- sizeA.z < objB.z && sizeA.z > objB.z) {
                return true;
            }
        }
    }
    return false;
}


simple3D_Pos_Double locilazePoint(Position3D_Double &playerPos, Position3D_Double &centrePoint, std::array<Vector3D_Double, 3> &unitVectorsCentre) {
    Vector3D_Double centPointVec = centrePoint.makeAVector(playerPos);
    return simple3D_Pos_Double(unitVectorsCentre[0].dotProduct(centPointVec), unitVectorsCentre[1].dotProduct(centPointVec), unitVectorsCentre[2].dotProduct(centPointVec));
}


bool boundBoxCol(Position3D_Double &playerPos, Position3D_Double &centrePoint, std::array<Vector3D_Double, 3> &unitVectorsCentre, simple3D_Pos_Double &playerSize, simple3D_Pos_Double &blockSize) {
    std::array<Position3D_Double, 9> points = {
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(playerSize.x, playerSize.y, playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(-playerSize.x, playerSize.y, playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(playerSize.x, -playerSize.y, playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(playerSize.x, playerSize.y, -playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(-playerSize.x, -playerSize.y, playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(playerSize.x, -playerSize.y, -playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(-playerSize.x, playerSize.y, -playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(-playerSize.x, -playerSize.y, -playerSize.z))),
        Position3D_Double(playerPos.changedBy(simple3D_Pos_Double(0, 0, 0)))
    };

    for (int i = 0; i < 9; i += 1) {
        if (AABBCCColision(locilazePoint(points[i], centrePoint, unitVectorsCentre), blockSize)) {
            return true;
        }
    }
    return false;
}


double findSmallestThree(std::array<double, 3> things) {
    double returningThing = things[0];
    for (int i = 0; i < 3; i += 1) {
        if (things[i] <= returningThing) {
            returningThing = things[i];
        }
    }
    return returningThing;
}


class Object3D_Double {
private:
    // links in style of having xxx and then another xxx ...
    // for now only cube collisions work

    std::vector<int> links;
    std::vector<Point_Double> points;
    std::vector<Position3D_Double> pseudoPos;
    std::vector<Position3D_Double> pseudoPosGradiant;
    simple3D_Pos_Double centrePoint;
    SimpleColor objectColor;
    int numsOfPoints;
    int numsOfFaces;
    bool stroked;
    SimpleColor outlineCol;
    simple3D_Pos_Double objectSize;
    double outLineSize;
    double outerRad;
    double innerRad;
    std::array<Vector3D_Double, 3> centreVec = {Vector3D_Double(simple3D_Pos_Double(1,0,0)), Vector3D_Double(simple3D_Pos_Double(0,1,0)), Vector3D_Double(simple3D_Pos_Double(0,0,1))};
    int firstPolygonNum;
    LODInfo myInfoLOD = LODInfo(true, 0.1);;

    void retypePolygonsThis(playerFullInfo &myInfo, std::vector<GlobalPolygon_Double> &globalPolygons, std::vector<LightSource> &allLights) {
        for (int i = 0; i < numsOfPoints; i += 1) {
            Position3D_Double oneLocalPoint = points[i].getRelativePos(myInfo.headingVector, myInfo.upVector, myInfo.rightVector, myInfo.originPoint);
            pseudoPos[i] = oneLocalPoint.makeIntoScreensCord(myInfo.cameraInfo);
            pseudoPosGradiant[i] = oneLocalPoint.makeIntoGradiantable(myInfo.cameraInfo);
        }

        int indexNum = firstPolygonNum;
        bool dontDrawOut = false;

        for (int i = 0; i < numsOfFaces; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 && pseudoPos[links[i+1]].myPos.z < 0.1 && pseudoPos[links[i+2]].myPos.z < 0.1) {
                dontDrawOut = true;
            }

            globalPolygons[indexNum].thisChange({points[links[i]], points[links[i+1]], points[links[i+2]]}, {pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]]},
                {pseudoPosGradiant[links[i]], pseudoPosGradiant[links[i+1]], pseudoPosGradiant[links[i+2]]},
                myInfo.cameraInfo, myInfoLOD.blockify , myInfoLOD.LODLevel, dontDrawOut);

            for (LightSource &oneLight : allLights) {
                oneLight.oneBlockUpdate(globalPolygons, indexNum);
            }

            indexNum += 1;

            dontDrawOut = false;
        }
    }

public:
    bool visibility;
    bool colision;

    Object3D_Double(int numOfPoints, int numFaces, std::vector<Point_Double> points, simple3D_Pos_Double &centre,
        std::vector<int> impLinks, std::vector<GlobalPolygon_Double> &globalPolygons, int &globalPolygonsPos, std::vector<LightSource> &allLights, playerFullInfo &currentInfo,
        simple3D_Pos_Double impSize = simple3D_Pos_Double(0,0,0),
        SimpleColor objectColor = SimpleColor(0,0,0,255),bool blockify = true, double detail = 0.1,
        SimpleColor outlineCol = SimpleColor(0,0,0,255), bool stroked = false, double outLineSize = 1, bool visibility = false, bool colision = false) {
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
        this->centreVec[0] = Vector3D_Double(simple3D_Pos_Double(1,0,0));
        this->centreVec[1] = Vector3D_Double(simple3D_Pos_Double(0,1,0));
        this->centreVec[2] = Vector3D_Double(simple3D_Pos_Double(0,0,1));
        this->firstPolygonNum = globalPolygonsPos;
        this->myInfoLOD = LODInfo(blockify, detail);

        for (int i = 0; i < numOfPoints; i += 1) {
            this->points.push_back(points[i]);
            this->points[i].setupUnitVectors(centrePoint);
            this->pseudoPos.push_back(Position3D_Double(simple3D_Pos_Double()));
            this->pseudoPosGradiant.push_back(Position3D_Double(simple3D_Pos_Double()));
        }

        for (int i = 0; i < numFaces; i += 3) {
            globalPolygons.push_back(GlobalPolygon_Double({Point_Double(simple3D_Pos_Double(0,0,0)),Point_Double(simple3D_Pos_Double(0,0,0)),Point_Double(simple3D_Pos_Double(0,0,0))},
                {Position3D_Double(simple3D_Pos_Double(0,0,0)),Position3D_Double(simple3D_Pos_Double(0,0,0)),Position3D_Double(simple3D_Pos_Double(0,0,0))}, {Position3D_Double(simple3D_Pos_Double(0,0,0)),Position3D_Double(simple3D_Pos_Double(0,0,0)),Position3D_Double(simple3D_Pos_Double(0,0,0))},
                objectColor, globalPolygonsPos, true));
            globalPolygonsPos += 1;
        }
        retypePolygonsThis(currentInfo, globalPolygons, allLights);

    }

    void retypePolygonsPlayer(playerFullInfo &myInfo, std::vector<GlobalPolygon_Double> &globalPolygons) {
        for (int i = 0; i < numsOfPoints; i += 1) {
            Position3D_Double oneLocalPoint = points[i].getRelativePos(myInfo.headingVector, myInfo.upVector, myInfo.rightVector, myInfo.originPoint);
            pseudoPos[i] = oneLocalPoint.makeIntoScreensCord(myInfo.cameraInfo);
            pseudoPosGradiant[i] = oneLocalPoint.makeIntoGradiantable(myInfo.cameraInfo);
        }


        int indexNum = firstPolygonNum;

        for (int i = 0; i < numsOfFaces; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 && pseudoPos[links[i+1]].myPos.z < 0.1 && pseudoPos[links[i+2]].myPos.z < 0.1) {
                indexNum += 1;
                continue;
            }

            globalPolygons[indexNum].playerChange({pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]]}, {pseudoPosGradiant[links[i]], pseudoPosGradiant[links[i+1]], pseudoPosGradiant[links[i+2]]},
                myInfo.cameraInfo, myInfoLOD.blockify , myInfoLOD.LODLevel, true);
            indexNum += 1;

        }
    }

    bool colide(Position3D_Double &playerPos, simple3D_Pos_Double &playerSize, double playerRad) {
        Position3D_Double centrePosition3D = Position3D_Double(centrePoint);
        double distance = std::pow(playerPos.myPos.x - centrePoint.x, 2) + std::pow(playerPos.myPos.y - centrePoint.y, 2) + std::pow(playerPos.myPos.z - centrePoint.z, 2);
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
        points[i].setPos(simple3D_Pos_Double(
                centrePoint.x + (points[i].xVector.x * objectSize.x) + (points[i].yVector.x * objectSize.y) + (points[i].zVector.x * objectSize.z),
                centrePoint.y + (points[i].xVector.y * objectSize.x) + (points[i].yVector.y * objectSize.y) + (points[i].zVector.y * objectSize.z),
                centrePoint.z + (points[i].xVector.z * objectSize.x) + (points[i].yVector.z * objectSize.y) + (points[i].zVector.z * objectSize.z)));
    }

    void rotates(double angleYZ, double angleXZ, double angleXY, playerFullInfo &playInfo, std::vector<GlobalPolygon_Double> &allPolygons, std::vector<LightSource> allLights) {
        double cosAngleYZ = std::cos(angleYZ);
        double sinAngleYZ = std::sin(angleYZ);
        double cosAngleXY = std::cos(angleXY);
        double sinAngleXY = std::sin(angleXY);
        double cosAngleXZ = std::cos(angleXZ);
        double sinAngleXZ = std::sin(angleXZ);
        centreVec[0] = Vector3D_Double(simple3D_Pos_Double(
                (centreVec[0].myPos.x * cosAngleXY - centreVec[0].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[0].myPos.z * sinAngleXZ,
                (centreVec[0].myPos.y * cosAngleXY + centreVec[0].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[0].myPos.z * sinAngleYZ,
                (centreVec[0].myPos.z * cosAngleXZ + centreVec[0].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[0].myPos.y * sinAngleYZ));

        centreVec[1] = Vector3D_Double(simple3D_Pos_Double(
                (centreVec[1].myPos.x * cosAngleXY - centreVec[1].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[1].myPos.z * sinAngleXZ,
                (centreVec[1].myPos.y * cosAngleXY + centreVec[1].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[1].myPos.z * sinAngleYZ,
                (centreVec[1].myPos.z * cosAngleXZ + centreVec[1].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[1].myPos.y * sinAngleYZ));

        centreVec[2] = Vector3D_Double(simple3D_Pos_Double(
                (centreVec[2].myPos.x * cosAngleXY - centreVec[2].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[2].myPos.z * sinAngleXZ,
                (centreVec[2].myPos.y * cosAngleXY + centreVec[2].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[2].myPos.z * sinAngleYZ,
                (centreVec[2].myPos.z * cosAngleXZ + centreVec[2].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[2].myPos.y * sinAngleYZ));

        for (int i = 0; i < numsOfPoints; i += 1) {
            // 1. XY rotation, 2. XZ rotation, 3. YZ rotation

            points[i].xVector = simple3D_Pos_Double(
                (points[i].xVector.x * cosAngleXY - points[i].xVector.y * sinAngleXY) * cosAngleXZ - points[i].xVector.z * sinAngleXZ,
                (points[i].xVector.y * cosAngleXY + points[i].xVector.x * sinAngleXY) * cosAngleYZ - points[i].xVector.z * sinAngleYZ,
                (points[i].xVector.z * cosAngleXZ + points[i].xVector.x * sinAngleXZ) * cosAngleYZ + points[i].xVector.y * sinAngleYZ);
            
            points[i].yVector = simple3D_Pos_Double(
                (points[i].yVector.x * cosAngleXY - points[i].yVector.y * sinAngleXY) * cosAngleXZ - points[i].yVector.z * sinAngleXZ,
                (points[i].yVector.y * cosAngleXY + points[i].yVector.x * sinAngleXY) * cosAngleYZ - points[i].yVector.z * sinAngleYZ,
                (points[i].yVector.z * cosAngleXZ + points[i].yVector.x * sinAngleXZ) * cosAngleYZ + points[i].yVector.y * sinAngleYZ);
            
            points[i].zVector = simple3D_Pos_Double(
                (points[i].zVector.x * cosAngleXY - points[i].zVector.y * sinAngleXY) * cosAngleXZ - points[i].zVector.z * sinAngleXZ,
                (points[i].zVector.y * cosAngleXY + points[i].zVector.x * sinAngleXY) * cosAngleYZ - points[i].zVector.z * sinAngleYZ,
                (points[i].zVector.z * cosAngleXZ + points[i].zVector.x * sinAngleXZ) * cosAngleYZ + points[i].zVector.y * sinAngleYZ);
            updatePoses(i);
            
        }
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    simple3D_Pos_Double getSize() {
        return objectSize;
    }

    simple3D_Pos_Double getPos() {
        return centrePoint;
    }

    void setPos(simple3D_Pos_Double changePos, playerFullInfo &playInfo, std::vector<GlobalPolygon_Double> &allPolygons, std::vector<LightSource> allLights) {
        centrePoint = changePos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void changePos(simple3D_Pos_Double changePos, playerFullInfo &playInfo, std::vector<GlobalPolygon_Double> &allPolygons, std::vector<LightSource> allLights) {
        centrePoint.x += changePos.x;
        centrePoint.y += changePos.y;
        centrePoint.z += changePos.z;

        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void setSize(simple3D_Pos_Double newSize, playerFullInfo &playInfo, std::vector<GlobalPolygon_Double> &allPolygons, std::vector<LightSource> allLights) {
        objectSize = newSize;
        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void changeSize(simple3D_Pos_Double newSize, playerFullInfo &playInfo, std::vector<GlobalPolygon_Double> &allPolygons, std::vector<LightSource> allLights) {
        objectSize.x += newSize.x;
        objectSize.y += newSize.y;
        objectSize.z += newSize.z;
        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void drawOutFastSDL2(playerFullInfo &myInfo, uint32_t* colorsBuffer, int pitch, double *zBuffer, std::vector<GlobalPolygon_Double> &globalPolygons) {
        int indexNum = firstPolygonNum;

        for (int i = 0; i < numsOfFaces; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 && pseudoPos[links[i+1]].myPos.z < 0.1 && pseudoPos[links[i+2]].myPos.z < 0.1) {
                continue;
            }

            globalPolygons[indexNum].drawOutSuperFast(zBuffer, colorsBuffer, pitch, myInfo.cameraInfo, stroked, outLineSize, myInfoLOD.blockify, myInfoLOD.LODLevel);
            indexNum += 1;

        }
    }
};


void createCube(simple3D_Pos_Double onePos, simple3D_Pos_Double oneSize, SimpleColor objColor, SimpleColor outColor, bool stroked, std::vector<GlobalPolygon_Double> &allPolygons, int &currentPolygon, std::vector<LightSource> &allLights,
    std::vector<Object3D_Double> &allObjects, playerFullInfo &currentPlayerInfo,
    double outlineSize, bool colisions = false, bool visibility = false, bool blockify = true, double lodLevel = 0.1) {

    simple3D_Pos_Double centre = simple3D_Pos_Double(onePos.x + (oneSize.x/2), onePos.y + (oneSize.y/2), onePos.z + (oneSize.z/2));

    allObjects.push_back(Object3D_Double(8, 36, {
        Point_Double(onePos.changedBy(0, 0, 0)),
        Point_Double(onePos.changedBy(oneSize.x, 0, 0)),
        Point_Double(onePos.changedBy(oneSize.x, oneSize.y, 0)),
        Point_Double(onePos.changedBy(oneSize.x, oneSize.y, oneSize.z)),
        Point_Double(onePos.changedBy(0, oneSize.y,oneSize.z)),
        Point_Double(onePos.changedBy(0, 0, oneSize.z)),
        Point_Double(onePos.changedBy(0, oneSize.y, 0)),
        Point_Double(onePos.changedBy(oneSize.x, 0, oneSize.z))
    },
        centre, {
         0,2,1, 0,6,2,  // Z=0
         3,4,5, 3,5,7,  // Z=1
         0,5,4, 0,4,6,  // X=0
         1,2,3, 1,3,7,  // X=1
         0,1,7, 0,7,5,  // Y=0
         2,4,3, 2,6,4   // Y=1
    },allPolygons, currentPolygon, allLights, currentPlayerInfo, oneSize, objColor, blockify, lodLevel, outColor, stroked, outlineSize, visibility, colisions));
}


void createLight(std::vector<LightSource> &allLights, std::vector<GlobalPolygon_Double> &allPolygons, LightTypes type, Position3D_Double impPos, int rayNumber, SimpleColor color,
    double intenzity, double lenghtDecay, Vector3D_Double directionRight = Vector3D_Double(simple3D_Pos_Double(0,0,0)), Vector3D_Double directionUp = Vector3D_Double(simple3D_Pos_Double(0,0,0)), double sourceHeight = 0, double sourceWidth = 0) {
    allLights.push_back(LightSource(type, impPos, rayNumber, color, allPolygons, intenzity, lenghtDecay, directionRight, directionUp, sourceHeight, sourceWidth));
}



std::vector<GlobalPolygon_Double> preparePolygonList() {
    std::vector<GlobalPolygon_Double> newList;
    return newList;
}

std::vector<LightSource> prepareLightList() {
    std::vector<LightSource> newList;
    return newList;
}

std::vector<Object3D_Double> prepareObjectList() {
    std::vector<Object3D_Double> newList;
    return newList;
}


struct basicInfo {
    std::vector<GlobalPolygon_Double> polygonList;
    int currePosPolygon = 0;
    std::vector<LightSource> lightSourcesList;
    std::vector<Object3D_Double> objectList;

    basicInfo() {
        this->polygonList = preparePolygonList();
        this->lightSourcesList = prepareLightList();
        this->objectList = prepareObjectList();
        this->currePosPolygon = 0;
    }
};



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
    bool pixelization;
    double detailLevel;
    double sizeRadius;
    bool colidingX = false;
    bool colidingY = false;
    bool colidingZ = false;
    simple3D_Pos_Double lastPos;
    simple3D_Pos_Double sizeBox;
    Position3D_Double nextPositionX;
    Position3D_Double nextPositionY;
    Position3D_Double nextPositionZ;
    std::array<double, 3> changes;
    double gravity;
    bool jumpingMode;
    double sestivity;

    void linearColisionSetup() {
        colidingX = false;
        colidingY = false;
        colidingZ = false;
        nextPositionX = Position3D_Double(simple3D_Pos_Double(myPos.myPos.x + changes[0], myPos.myPos.y, myPos.myPos.z));
        nextPositionY = Position3D_Double(simple3D_Pos_Double(myPos.myPos.x, myPos.myPos.y + changes[1], myPos.myPos.z));
        nextPositionZ = Position3D_Double(simple3D_Pos_Double(myPos.myPos.x, myPos.myPos.y, myPos.myPos.z + changes[2]));
    }

public:
    playerFullInfo myBasicInfo;

    Player_Double(double speed = 0.5, int screenHeight = 1080, int screenWidth = 1920, int fov = 90, simple3D_Pos_Double beginPos = simple3D_Pos_Double(0,0,0),
        bool pixelization = true, double detaiLevel = 1, simple3D_Pos_Double sizeBox = simple3D_Pos_Double(4,4,4), double gravity = 0, bool jumpingMode = false, double senstivity = 0.001) {
        this->myPos.setPosition(beginPos);
        this->speed = speed;
        this->pixelization = pixelization;
        this->detailLevel = detaiLevel;
        this->sizeRadius = findSmallestThree({sizeBox.x, sizeBox.y, sizeBox.z});
        this->sizeBox = sizeBox;
        this->colidingX = false;
        this->colidingY = false;
        this->colidingZ = false;
        this->nextPositionX = Position3D_Double(beginPos);
        this->nextPositionY = Position3D_Double(beginPos);
        this->nextPositionZ = Position3D_Double(beginPos);
        this->changes = {0,0,0};
        this->gravity = gravity;
        this->jumpingMode = jumpingMode;
        this->sestivity = senstivity;

        this->values = playerHelpfulVals(0,0);
        this->valuesUp = playerHelpfulVals(this->values.angleY,(this->values.angleZ - M_PI / 2));
        this->valuesRight = playerHelpfulVals((this->values.angleY + M_PI / 2),this->values.angleZ);

        this->headingVec.setVector(simple3D_Pos_Double(this->values.cosAngleY * this->values.cosAngleZ, this->values.sinAngleY * this->values.cosAngleZ, this->values.sinAngleZ));
        this->rightVec.setVector(simple3D_Pos_Double(this->valuesRight.cosAngleY * this->valuesRight.cosAngleZ, this->valuesRight.sinAngleY * this->valuesRight.cosAngleZ, 0));
        this->upVec.setVector(simple3D_Pos_Double(this->valuesUp.cosAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleZ));

        double radiansFOV = fov * (M_PI / 180);
        this->myCameraInfo = screenAndCameraInfo(((screenWidth / 2) / (std::tan(radiansFOV/2))) * 1.33, ((screenHeight / 2) / (std::tan(radiansFOV/2))), screenHeight, screenWidth);
        this->myBasicInfo = playerFullInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
    }

    void camera(uint32_t* sdlBuffer, double* zBuffer, int pitch, basicInfo &globalInfo) {
        pitch /= 4;
        linearColisionSetup();
        for (Object3D_Double &oneObject : globalInfo.objectList) {
            if (oneObject.visibility) {
                oneObject.drawOutFastSDL2(myBasicInfo, sdlBuffer, pitch, zBuffer, globalInfo.polygonList);
            }
            if (oneObject.colision) {
                if (oneObject.colide(nextPositionX, sizeBox, sizeRadius)) {
                    colidingX = true;
                }
                if (oneObject.colide(nextPositionY, sizeBox, sizeRadius)) {
                    colidingY = true;
                }
                if (oneObject.colide(nextPositionZ, sizeBox, sizeRadius)) {
                    colidingZ = true;
                }
            }
        }
    }

    void cameraMovementSDL2(SDL_Event event, basicInfo &globalInfo) {
        if (event.type == SDL_MOUSEMOTION) {
            values.changeAngleY(event.motion.xrel * sestivity);
            valuesUp.changeAngleY(event.motion.xrel * sestivity);
            valuesRight.changeAngleY(event.motion.xrel * sestivity);

            if (values.angleZ >= -1.4 && -event.motion.yrel < 0) {
                values.changeAngleZ(-event.motion.yrel * sestivity);
                valuesUp.changeAngleZ(-event.motion.yrel * sestivity);
            }
            if (values.angleZ <= 1 && -event.motion.yrel > 0) {
                values.changeAngleZ(-event.motion.yrel * sestivity);
                valuesUp.changeAngleZ(-event.motion.yrel * sestivity);
            }

            headingVec.setVector(simple3D_Pos_Double(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_Double(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_Double(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
            myBasicInfo.changeInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
            for (Object3D_Double &oneObject : globalInfo.objectList) {
                oneObject.retypePolygonsPlayer(myBasicInfo, globalInfo.polygonList);
            }
        }
    }

    void defaultMovementSDL2(const Uint8* state) {
        changes[0] = 0;
        changes[1] = 0;
        changes[2] = 0;
        if (state[SDL_SCANCODE_W]) {
            changes[0] += speed * values.cosAngleY;
            changes[1] += speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_S]) {
            changes[0] += -speed * values.cosAngleY;
            changes[1] += -speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_D]) {
            changes[0] += speed * valuesRight.cosAngleY;
            changes[1] += speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_A]) {
            changes[0] += -speed * valuesRight.cosAngleY;
            changes[1] += -speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_SPACE]) {
            changes[2] += speed;
        }
        if (state[SDL_SCANCODE_LSHIFT]) {
            changes[2] += -speed;
        }
    }

    void gravityMovement(const Uint8* state) {
        changes[0] = 0;
        changes[1] = 0;
        if (std::abs(changes[2]) <= gravity * -10) {
            changes[2] += gravity;
        }
        else {
            changes[2] = gravity * 10;
        }
        if (state[SDL_SCANCODE_W]) {
            changes[0] += speed * values.cosAngleY;
            changes[1] += speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_S]) {
            changes[0] += -speed * values.cosAngleY;
            changes[1] += -speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_D]) {
            changes[0] += speed * valuesRight.cosAngleY;
            changes[1] += speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_A]) {
            changes[0] += -speed * valuesRight.cosAngleY;
            changes[1] += -speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_SPACE] && colidingZ) {
            changes[2] += speed*20;
        }
    }

    void movementSDL2(basicInfo &globalInfo, const Uint8* stateKeyboard) {
        if (!colidingX) {
            myPos.myPos.x += changes[0];
        }
        if (colidingX) {
            changes[0] = 0;
        }

        if (!colidingY) {
            myPos.myPos.y += changes[1];
        }
        if (colidingY) {
            changes[1] = 0;
        }

        if (!colidingZ) {
            myPos.myPos.z += changes[2];
        }
        if (colidingZ) {
            changes[2] = 0;
        }

        if (!colidingX || !colidingY || !colidingZ) {
            myBasicInfo.changeInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
            for (Object3D_Double &oneObject : globalInfo.objectList) {
                oneObject.retypePolygonsPlayer(myBasicInfo, globalInfo.polygonList);
            }
        }

        if (!jumpingMode) {
            defaultMovementSDL2(stateKeyboard);
        }
        if (jumpingMode) {
            gravityMovement(stateKeyboard);
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


class gameInfo {
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* buffer;
    double* zBuffer;
    uint32_t* colorBuffer;
    int pitch;

public:

    double renderDistance;
    basicInfo gameGlobals;
    int height, width;
    uint32_t backgroundColor;
    const Uint8* myState = SDL_GetKeyboardState(NULL);
    const Uint32 mouseState = SDL_GetMouseState(NULL,NULL);

    gameInfo(int windowWidth, int windowHeight, char *windowName, double renderDistance, SimpleColor backgroundColor) {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cout << "SDL hasn`t inicilized, error code: " << SDL_GetError() << std::endl;
        }
        this->width = windowWidth;
        this->height = windowHeight;
        this->window = SDL_CreateWindow(windowName, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width, height,SDL_WINDOW_OPENGL);
        if (!this->window) {
            std::cout << "Window hasn`t opened, error code: " << SDL_GetError() << std::endl;
        }
        this->renderer = SDL_CreateRenderer(this->window, -1, SDL_RENDERER_ACCELERATED);

        this->buffer = SDL_CreateTexture(this->renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        width, height);
        this->zBuffer = (double*)malloc(windowHeight * windowWidth * sizeof(double));
        std::fill(zBuffer, zBuffer + windowWidth * windowHeight, renderDistance);
        this->renderDistance = renderDistance;
        this->colorBuffer = nullptr;
        this->backgroundColor = backgroundColor.convertToBinary();
    }

    void drawScene(Player_Double &player) {
        SDL_RenderClear(renderer);

        SDL_LockTexture(buffer, NULL, (void**)&colorBuffer, &pitch);

        std::fill(colorBuffer, colorBuffer + height * width, backgroundColor);
        std::fill(zBuffer, zBuffer + height * width, renderDistance);

        player.camera(colorBuffer, zBuffer, pitch, gameGlobals);

        SDL_UnlockTexture(buffer);
        SDL_RenderCopy(renderer, buffer, NULL, NULL);

        SDL_RenderPresent(renderer);
    }

    void end() {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        std::free(zBuffer);
        SDL_Quit();
    }

    ~gameInfo() {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        std::free(zBuffer);
        SDL_Quit();
    }
};


int main(int argc, char* argv[]) {
    bool running = true;

    gameInfo game = gameInfo(2560, 1440, "Super hra", 800, SimpleColor(0,0,255));

    SDL_Event event;
    SDL_SetRelativeMouseMode(SDL_TRUE);

    Player_Double myPlayer = Player_Double(0.5, 1440, 2560, 90, simple3D_Pos_Double(10,10,10));

    for (int i = 0; i < 22; i += 1) {
        createCube(simple3D_Pos_Double(getRandomDouble(-35,35),getRandomDouble(-35,35),getRandomDouble(-35,35)), simple3D_Pos_Double(getRandomDouble(2,15),getRandomDouble(2,15),getRandomDouble(2,15)),
            SimpleColor(getRandomInt(0,50),getRandomInt(0,50),getRandomInt(0,50)), SimpleColor(0,0,0), false,
        game.gameGlobals.polygonList, game.gameGlobals.currePosPolygon, game.gameGlobals.lightSourcesList, game.gameGlobals.objectList, myPlayer.myBasicInfo, 20, true, true, true, 0.5);
    }

    createCube(simple3D_Pos_Double(-10,-10,-5), simple3D_Pos_Double(50,50,2),
            SimpleColor(0,235,0), SimpleColor(0,0,0), false,
        game.gameGlobals.polygonList, game.gameGlobals.currePosPolygon, game.gameGlobals.lightSourcesList, game.gameGlobals.objectList, myPlayer.myBasicInfo, 20, false, true, true, 0.5);

    createLight(game.gameGlobals.lightSourcesList, game.gameGlobals.polygonList, LightTypes::paralel, Position3D_Double(simple3D_Pos_Double(0,5,5)),
        100, SimpleColor(255,255,255), 0.2, 1, Vector3D_Double(simple3D_Pos_Double(0,1,0)), Vector3D_Double(simple3D_Pos_Double(0,0,1)), 5, 5);

    createCube(simple3D_Pos_Double(10,2,5), simple3D_Pos_Double(1,1,2),
            SimpleColor(0,200,0), SimpleColor(0,0,0), false,
        game.gameGlobals.polygonList, game.gameGlobals.currePosPolygon, game.gameGlobals.lightSourcesList, game.gameGlobals.objectList, myPlayer.myBasicInfo, 20, false, true, true, 0.5);

    createCube(simple3D_Pos_Double(-10,2,5), simple3D_Pos_Double(1,1,2),
            SimpleColor(200,200,0), SimpleColor(0,0,0), false,
        game.gameGlobals.polygonList, game.gameGlobals.currePosPolygon, game.gameGlobals.lightSourcesList, game.gameGlobals.objectList, myPlayer.myBasicInfo, 20, false, true, true, 0.5);

    while (running) {
        while (SDL_PollEvent(&event)) {
            myPlayer.cameraMovementSDL2(event, game.gameGlobals);
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                if (event.key.keysym.sym == SDLK_k) {
                    game.gameGlobals.lightSourcesList[0].changePos(simple3D_Pos_Double(10,1,5), game.gameGlobals.polygonList,
                        Vector3D_Double(simple3D_Pos_Double(0,1,0)), Vector3D_Double(simple3D_Pos_Double(0,0,1)));
                }
                if (event.key.keysym.sym == SDLK_l) {
                    game.gameGlobals.lightSourcesList[0].changePos(simple3D_Pos_Double(-10,1,5), game.gameGlobals.polygonList,
                        Vector3D_Double(simple3D_Pos_Double(0,1,0)), Vector3D_Double(simple3D_Pos_Double(0,0,1)));
                }
                if (event.key.keysym.sym == SDLK_m) {
                    game.gameGlobals.lightSourcesList[0].changePos(simple3D_Pos_Double(-100,1,5), game.gameGlobals.polygonList,
                        Vector3D_Double(simple3D_Pos_Double(0,1,0)), Vector3D_Double(simple3D_Pos_Double(0,0,1)));
                }
            }
        }
        myPlayer.movementSDL2(game.gameGlobals, game.myState);
        game.drawScene(myPlayer);
    }

    game.end();

    return 0;
}