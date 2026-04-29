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

#define MonitorHeight 1440
#define MonitorWidth 2560

const Uint8* state = SDL_GetKeyboardState(NULL);
const Uint32 mouseState = SDL_GetMouseState(NULL,NULL);

//fully operational 3D raycasting engine

double getRandomDouble(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
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

        return Position3D_Double(simple3D_Pos_Double(-10, -10, -15));
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

    void drawOutPolygonSDL2Fast(double* zBufferImp, uint32_t* colorsBuffer, int pitch, screenAndCameraInfo const &cameraInfo, bool outLine, SimpleColor outLineCol, int outlineThickness, bool blockification, double blockDetail) {
        std::array<int, 4> minsAmaxs = minsAndMaxs(cameraInfo);
        std::array<double, 2> gradiant = getGradiantsDouble(points[0], points[1], points[2]);

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
                    double globalZ = points[2].myPos.z + (xPos - points[2].myPos.x) * gradiant[0] + (yPos - points[2].myPos.y) * gradiant[1];

                    if (blockification) {
                        for (int yPosReal = yPos; yPosReal < yPos + blockyfacion; yPosReal += 1) {
                            if (yPosReal >= cameraInfo.screenHeight) {
                                yPosReal += blockyfacion;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] ) {
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

                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] ) {
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
                    maxY = cameraInfo.screenHeight-2;
                }

                if (minY >= cameraInfo.screenHeight) {
                    minY = cameraInfo.screenHeight-2;
                }

                if (maxY <= 0) {
                    maxY = 0;
                }

                for (int yPos = minY; yPos < maxY+1; yPos += 1) {
                    double globalZ = points[2].myPos.z + (xPos - points[2].myPos.x) * gradiant[0] + (yPos - points[2].myPos.y) * gradiant[1];

                    if (blockification) {
                        for (int xPosReal = xPos; xPosReal < xPos + blockyfacion; xPosReal += 1) {
                            if (xPosReal >= cameraInfo.screenWidth) {
                                xPosReal += blockyfacion;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] ) {
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
                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] ) {
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
        std::array<double, 2> gradiant = getGradiantsDouble(points[0], points[1], points[2]);

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

public:
    bool visibility;
    bool colision;

    Object3D_Double(int numOfPoints, int numFaces, std::vector<Point_Double> &points, simple3D_Pos_Double &centre,
        std::vector<int> impLinks, simple3D_Pos_Double impSize = simple3D_Pos_Double(0,0,0), SimpleColor objectColor = SimpleColor(0,0,0,255),
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

        for (int i = 0; i < numOfPoints; i += 1) {
            this->points.push_back(points[i]);
            this->points[i].setupUnitVectors(centrePoint);
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

    void rotates(double angleYZ, double angleXZ, double angleXY) {
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
    }

    simple3D_Pos_Double getSize() {
        return objectSize;
    }

    simple3D_Pos_Double getPos() {
        return centrePoint;
    }

    void setPos(simple3D_Pos_Double changePos) {
        centrePoint = changePos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
    }

    void changePos(simple3D_Pos_Double changePos) {
        centrePoint.x += changePos.x;
        centrePoint.y += changePos.y;
        centrePoint.z += changePos.z;

        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
    }

    void setSize(simple3D_Pos_Double newSize) {
        objectSize = newSize;
        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
    }

    void changeSize(simple3D_Pos_Double newSize) {
        objectSize.x += newSize.x;
        objectSize.y += newSize.y;
        objectSize.z += newSize.z;
        for (int i = 0; i < numsOfPoints; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
    }

    void drawOutSDL2(Vector3D_Double &headingVec, Vector3D_Double &upVector, Vector3D_Double &rightVector, Position3D_Double &headingOrigin, screenAndCameraInfo &camera_info, uint32_t* colorsBuffer, int pitch, double *zBuffer, bool blockify, double detail) {

        std::vector<Position3D_Double> pseudoPos;

        for (int i = 0; i < numsOfPoints; i += 1) {
            pseudoPos.push_back((points[i].getRelativePos(headingVec, upVector, rightVector, headingOrigin)).makeIntoScreensCord(camera_info));
        }

        for (int i = 0; i < numsOfFaces; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0 || pseudoPos[links[i+1]].myPos.z < 0 || pseudoPos[links[i+2]].myPos.z < 0) {
                continue;
            }

            ScreenPolygon_Double onePolygon = ScreenPolygon_Double(pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]], objectColor);
            onePolygon.drawOutPolygonSDL2Fast(zBuffer, colorsBuffer, pitch, camera_info, stroked, outlineCol, outLineSize, blockify, detail);
        }
    }
};


Object3D_Double createCubePoints(simple3D_Pos_Double onePos, simple3D_Pos_Double oneSize, SimpleColor objColor, SimpleColor outColor, bool stroked, double outlineSize, bool colisions = false, bool visibility = false) {
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
    }, oneSize, objColor, outColor, stroked, outlineSize, visibility, colisions);
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
    bool pixelization;
    double detailLevel;
    double sizeRadius;
    bool coliding = false;
    simple3D_Pos_Double lastPos;
    simple3D_Pos_Double sizeBox;
    Position3D_Double nextPosition;
    double gravity;
    bool jumpingMode;
    double sestivity;

public:

    Player_Double(double speed = 0.5, int screenHeight = 1080, int screenWidth = 1920, int fov = 90, simple3D_Pos_Double beginPos = simple3D_Pos_Double(0,0,0),
        bool pixelization = true, double detaiLevel = 1, simple3D_Pos_Double sizeBox = simple3D_Pos_Double(4,4,4), double gravity = 0, bool jumpingMode = false, double senstivity = 0.001) {
        this->myPos.setPosition(beginPos);
        this->speed = speed;
        this->pixelization = pixelization;
        this->detailLevel = detaiLevel;
        this->sizeRadius = findSmallestThree({sizeBox.x, sizeBox.y, sizeBox.z});
        this->sizeBox = sizeBox;
        this->coliding = false;
        this->nextPosition = Position3D_Double(beginPos);
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
    }

    void camera(std::vector<Object3D_Double> const &objects, uint32_t* sdlBuffer, double* zBuffer, SimpleColor* universalColorBuffer, int pitch) {
        pitch /= 4;
        coliding = false;
        for (Object3D_Double oneObject : objects) {
            if (oneObject.visibility) {
                oneObject.drawOutSDL2(headingVec, upVec, rightVec, myPos, myCameraInfo, sdlBuffer, pitch, zBuffer, pixelization, detailLevel);
            }
            if (oneObject.colision) {
                if (oneObject.colide(nextPosition, sizeBox, sizeRadius)) {
                    coliding = true;
                }
            }
        }
    }

    void cameraMovementSDL2(SDL_Event event) {
        if (event.type == SDL_MOUSEMOTION) {
            values.changeAngleY(event.motion.xrel * sestivity);
            valuesUp.changeAngleY(event.motion.xrel * sestivity);
            valuesRight.changeAngleY(event.motion.xrel * sestivity);

            if (values.angleZ >= -1.4 && values.angleZ <= 1) {
                values.changeAngleZ(-event.motion.yrel * sestivity);
                valuesUp.changeAngleZ(-event.motion.yrel * sestivity);
            }

            headingVec.setVector(simple3D_Pos_Double(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_Double(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_Double(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));

        }
    }

    void defaultMovementSDL2() {
        if (state[SDL_SCANCODE_W]) {
            nextPosition.myPos.x += speed * values.cosAngleY;
            nextPosition.myPos.y += speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_S]) {
            nextPosition.myPos.x -= speed * values.cosAngleY;
            nextPosition.myPos.y -= speed * values.sinAngleY;
        }
        if (state[SDL_SCANCODE_D]) {
            nextPosition.myPos.x += speed * valuesRight.cosAngleY;
            nextPosition.myPos.y += speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_A]) {
            nextPosition.myPos.x -= speed * valuesRight.cosAngleY;
            nextPosition.myPos.y -= speed * valuesRight.sinAngleY;
        }
        if (state[SDL_SCANCODE_SPACE]) {
            nextPosition.myPos.z += speed;
        }
        if (state[SDL_SCANCODE_LSHIFT]) {
            nextPosition.myPos.z -= speed;
        }
    }

    void movementSDL2() {
        if (!coliding) {
            myPos = nextPosition;
        }
        if (coliding) {
            nextPosition = myPos;
        }
        defaultMovementSDL2();
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
    SDL_SetRelativeMouseMode(SDL_TRUE);

    Player_Double firstPlayer = Player_Double(0.5,MonitorHeight,MonitorWidth,90, simple3D_Pos_Double(0,0,0), true, 0.1);

    std::vector<Object3D_Double> kostky = {};
    for (int i = 0; i < 12; i += 1) {
        kostky.push_back(createCubePoints(simple3D_Pos_Double(getRandomDouble(-35,35),getRandomDouble(-35,35),getRandomDouble(-35,35)), simple3D_Pos_Double(getRandomDouble(2,10),getRandomDouble(2,10),getRandomDouble(2,10)),
            SimpleColor(getRandomInt(1,255),getRandomInt(1,255),getRandomInt(1,255)), SimpleColor(0, 0, 0), false, 50, true, true));
    }

    kostky.push_back(createCubePoints(simple3D_Pos_Double(5,10,3), simple3D_Pos_Double(2,3,1), SimpleColor(250,0,0), SimpleColor(0, 0, 0), false, 50, true, true));
    kostky.push_back(createCubePoints(simple3D_Pos_Double(2,15,13), simple3D_Pos_Double(2,3,1), SimpleColor(250,0,0), SimpleColor(0, 0, 0), false, 50, true, true));
    kostky.push_back(createCubePoints(simple3D_Pos_Double(12,15,13), simple3D_Pos_Double(2,3,3), SimpleColor(250,120,0), SimpleColor(0, 0, 0), false, 50, true, true));

    double renderDistance = 800;

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
            firstPlayer.cameraMovementSDL2(event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                if (event.key.keysym.sym == SDLK_j) {
                    kostky[getRandomInt(0,4)].rotates(0,0.5,0);
                }
                if (event.key.keysym.sym == SDLK_i) {
                    kostky[getRandomInt(0,4)].rotates(0,-0.5,0);
                }
                if (event.key.keysym.sym == SDLK_k) {
                    kostky[getRandomInt(0,4)].rotates(0,0,0.5);
                }
                if (event.key.keysym.sym == SDLK_l) {
                    kostky[getRandomInt(0,4)].rotates(0,0,-0.5);
                }
                if (event.key.keysym.sym == SDLK_m) {
                    kostky[getRandomInt(0,4)].rotates(0.5
                        ,0,0);
                }
                if (event.key.keysym.sym == SDLK_n) {
                    kostky[getRandomInt(0,4)].rotates(-0.5,0,0);
                }
                if (event.key.keysym.sym == SDLK_c) {
                    kostky[getRandomInt(0,4)].changeSize(simple3D_Pos_Double(0,1,0));
                }
                if (event.key.keysym.sym == SDLK_v) {
                    kostky[getRandomInt(0,4)].changeSize(simple3D_Pos_Double(1,0,0));
                }
                if (event.key.keysym.sym == SDLK_b) {
                    kostky[getRandomInt(0,4)].changeSize(simple3D_Pos_Double(0,0,1));
                }
            }
        }

        firstPlayer.movementSDL2();

        SDL_RenderClear(renderer);

        SDL_LockTexture(buffer, NULL, (void**)&nasPixelBuffer, &pitch);

        std::fill(nasPixelBuffer, nasPixelBuffer + MonitorHeight * MonitorWidth, backgroundColor);
        std::fill(zBuffer, zBuffer + MonitorHeight * MonitorWidth, renderDistance);

        firstPlayer.camera(kostky, nasPixelBuffer, zBuffer, bufferColorMy, pitch);

        SDL_UnlockTexture(buffer);
        SDL_RenderCopy(renderer, buffer, NULL, NULL);

        SDL_RenderPresent(renderer);
    }


    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::free(zBuffer);


    return 0;
}