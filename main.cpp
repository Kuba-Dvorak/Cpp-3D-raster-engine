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

#define MonitorHeight 1440
#define MonitorWidth 2560

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

    void changePosition(simple3D_Pos_Double impPos) {
        myPos.x += impPos.x;
        myPos.y += impPos.y;
        myPos.z += impPos.z;
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
            minsAMax[0] = camerasInfo.screenWidth;
        }
        if (minsAMax[1] >= camerasInfo.screenHeight) {
            minsAMax[1] = camerasInfo.screenHeight;
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

            for (int yPos = minsAmaxs[3]; yPos < minsAmaxs[1]; yPos += blockyfacion) {
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

                for (int xPos = minX; xPos < maxX; xPos += 1) {
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

            for (int xPos = minsAmaxs[2]; xPos < minsAmaxs[0]; xPos += blockyfacion) {

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

                for (int yPos = minY; yPos < maxY; yPos += 1) {
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

    Point_Double(simple3D_Pos_Double impPosition) {
        this->position.myPos = impPosition;
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
    std::vector<simple3D_Pos_Double> originalPoses;

public:

    Object3D_Double(int numOfPoints, int numFaces, std::vector<Point_Double> &points, simple3D_Pos_Double &centre, std::vector<int> impLinks, simple3D_Pos_Double impSize, SimpleColor objectColor, SimpleColor outlineCol, bool stroked) {
        this->stroked = stroked;
        this->outlineCol = outlineCol;
        this->objectColor = objectColor;
        this->numsOfFaces = numFaces;
        this->numsOfPoints = numOfPoints;
        this->centrePoint = centre;
        this->objectSize.x = std::sqrt(impSize.x)/64;
        this->objectSize.y = std::sqrt(impSize.y)/64;
        this->objectSize.z = std::sqrt(impSize.z)/64;
        this->links = impLinks;

        Position3D_Double centrePointPosition = Position3D_Double(centrePoint);

        for (int i = 0; i < numOfPoints; i += 1) {
            this->points.push_back(points[i]);
            this->originalPoses.push_back(points[i].getPos().myPos);

            Position3D_Double onePos = points[i].getPos();
            this->vectorsFromCentre.push_back(centrePointPosition.makeAUnitVector(onePos));
        }
    }

    void rotates(double angleYZ, double angleXZ, double angleXY) {
        double cosAngleYZ = std::cos(angleYZ);
        double sinAngleYZ = std::sin(angleYZ);
        double cosAngleXY = std::cos(angleXY);
        double sinAngleXY = std::sin(angleXY);
        double cosAngleXZ = std::cos(angleXZ);
        double sinAngleXZ = std::sin(angleXZ);

        for (int i = 0; i < numsOfPoints; i += 1) {
            // 1. XY rotation, 2. XZ rotation, 3. YZ rotation

            Position3D_Double onePos = points[i].getPos();
            simple3D_Pos_Double centrePointCentric = simple3D_Pos_Double(onePos.myPos.x - centrePoint.x, onePos.myPos.y - centrePoint.y, onePos.myPos.z - centrePoint.z);
            originalPoses[i] = simple3D_Pos_Double(
                centrePoint.x + ((centrePointCentric.x * cosAngleXY - centrePointCentric.y * sinAngleXY) * cosAngleXZ - centrePointCentric.z * sinAngleXZ),
                centrePoint.y + ((centrePointCentric.y * cosAngleXY + centrePointCentric.x * sinAngleXY) * cosAngleYZ - centrePointCentric.z * sinAngleYZ),
                centrePoint.z + ((centrePointCentric.z * cosAngleXZ + centrePointCentric.x * sinAngleXZ) * cosAngleYZ + centrePointCentric.y * sinAngleYZ));
            points[i].setPos(originalPoses[i]);
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
            points[i].setPos(changePos);
        }
    }

    void changePos(simple3D_Pos_Double changePos) {
        centrePoint.x += changePos.x;
        centrePoint.y += changePos.y;
        centrePoint.z += changePos.z;

        for (int i = 0; i < numsOfPoints; i += 1) {
            points[i].changePos(changePos);
        }
    }

    void setSize(simple3D_Pos_Double newSize) {
        objectSize = newSize;
        for (int i = 0; i < numsOfPoints; i += 1) {
            simple3D_Pos_Double oneVec = vectorsFromCentre[i].getVec();
            simple3D_Pos_Double onePos = originalPoses[i];

            points[i].setPos(simple3D_Pos_Double( onePos.x + (oneVec.x * objectSize.x), onePos.y + (oneVec.y * objectSize.y), onePos.z + (oneVec.z * objectSize.z)));
        }
    }

    void changeSize(simple3D_Pos_Double newSize) {
        objectSize.x += newSize.x;
        objectSize.y += newSize.y;
        objectSize.z += newSize.z;
        for (int i = 0; i < numsOfPoints; i += 1) {
            simple3D_Pos_Double oneVec =  vectorsFromCentre[i].getVec();
            simple3D_Pos_Double onePos = originalPoses[i];

            points[i].setPos(simple3D_Pos_Double( onePos.x + (oneVec.x * objectSize.x), onePos.y + (oneVec.y * objectSize.y), onePos.z + (oneVec.z * objectSize.z)));
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
            if (pseudoPos[links[i]].myPos.z < 0 || pseudoPos[links[i+1]].myPos.z < 0 || pseudoPos[links[i+2]].myPos.z < 0) {
                continue;
            }

            ScreenPolygon_Double onePolygon = ScreenPolygon_Double(pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]], objectColor);
            onePolygon.drawOutPolygonSDL2Fast(zBuffer, colorsBuffer, pitch, camera_info, stroked, outlineCol, 25, true, 1);
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
        this->myCameraInfo = screenAndCameraInfo(((screenWidth / 2) / (std::tan(radiansFOV/2))) * 1.33, ((screenHeight / 2) / (std::tan(radiansFOV/2))), screenHeight, screenWidth);
    }

    void camera(std::vector<Object3D_Double> const &objects, uint32_t* sdlBuffer, double* zBuffer, SimpleColor* universalColorBuffer, bool sdl2Type, int pitch) {
        pitch /= 4;
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
            if (values.angleZ >= -1.4) {
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

    Player_Double firstPlayer = Player_Double(0.5,MonitorHeight,MonitorWidth,90);

    std::vector<Object3D_Double> kostky = {};
    for (int i = 0; i < 6; i += 1) {
        kostky.push_back(createCubePoints(simple3D_Pos_Double(getRandomDouble(-35,35),getRandomDouble(-35,35),getRandomDouble(-35,35)), simple3D_Pos_Double(getRandomDouble(2,10),getRandomDouble(2,10),getRandomDouble(2,10)),
            SimpleColor(getRandomInt(1,255),getRandomInt(1,255),getRandomInt(1,255)), SimpleColor(0, 0, 0), false));
    }

    kostky.push_back(createCubePoints(simple3D_Pos_Double(5,10,3), simple3D_Pos_Double(2,3,1), SimpleColor(250,0,0), SimpleColor(0, 0, 0), false));
    kostky.push_back(createCubePoints(simple3D_Pos_Double(2,15,13), simple3D_Pos_Double(2,3,1), SimpleColor(250,0,0), SimpleColor(0, 0, 0), false));
    kostky.push_back(createCubePoints(simple3D_Pos_Double(12,15,13), simple3D_Pos_Double(2,3,3), SimpleColor(250,120,0), SimpleColor(0, 0, 0), false));

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
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                if (event.key.keysym.sym == SDLK_j) {
                    kostky[getRandomInt(0,5)].rotates(0,0.5,0);
                }
                if (event.key.keysym.sym == SDLK_i) {
                    kostky[getRandomInt(0,5)].rotates(0,-0.5,0);
                }
                if (event.key.keysym.sym == SDLK_k) {
                    kostky[getRandomInt(0,5)].rotates(0,0,0.5);
                }
                if (event.key.keysym.sym == SDLK_l) {
                    kostky[getRandomInt(0,5)].rotates(0,0,-0.5);
                }
                if (event.key.keysym.sym == SDLK_m) {
                    kostky[getRandomInt(0,5)].rotates(0.5
                        ,0,0);
                }
                if (event.key.keysym.sym == SDLK_n) {
                    kostky[getRandomInt(0,5)].rotates(-0.5,0,0);
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