#include <iostream>
#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <cmath>
#include <regex>
#include <algorithm>

#define MonitorHeight 1080
#define MonitorWidth 1920

const Uint8* state = SDL_GetKeyboardState(NULL);

//fully operational 3D raycasting engine

float getRandomFloat(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    float returnableValue = float((dis(gen)*(max-min+1))+min);
    return returnableValue;
}

int getRandomInt(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    int returnableValue = int((dis(gen)*(max-min+1))+min);
    return returnableValue;
}

float absoluteValueVector(std::vector<float> &myVector) {
    return float(std::sqrt(std::pow(myVector[0],2)+std::pow(myVector[1],2)+std::pow(myVector[2],2)));
}

float findAbsoluteDistance(std::vector<float> &myObjA, std::vector<float> &myObjB) {
    float dx = myObjA[0] - myObjB[0];
    float dy = myObjA[1] - myObjB[1];
    float dz = myObjA[2] - myObjB[2];
    return std::sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
}

float findSmallestFloat(std::vector<float> &myList) {
    float minimum = myList[0];
    for (float thing : myList) {
        if (thing <= minimum) {
            minimum = thing;
        }
    }
    return minimum;
}


std::vector<int> thingsForLine(std::vector<int> &myObjA, std::vector<int> &myObjB) {
    int dx = myObjB[0] - myObjA[0];
    int dy = myObjB[1] - myObjA[1];
    int c =  int(std::round(std::sqrt(pow(dx,2)+pow(dy,2))));
    if (c == 0) {
        return {(dx * 100), (dy * 100), 0};
    }
    return {int(dx/c),int(dy/c),c};
}


int findSmallestFloatIndex(std::vector<float> &myList) {
    float minimum = myList[0];
    int indexMin = 0;
    for (int i = 0; i < myList.size(); i += 1) {
        if (myList[i] <= minimum) {
            minimum = myList[i];
            indexMin = i;
        }
    }
    return indexMin;
}

std::vector<float> unitCircle(std::vector<float> &myObjA, std::vector<float> &myObjB) {
    float dx = myObjA[0] - myObjB[0];
    float dy = myObjA[1] - myObjB[1];
    float dz = myObjA[2] - myObjB[2];
    float c =  std::sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
    return {dx/c,dy/c,dz/c,c};
}

bool abbccColShort(std::vector<float> objAPos, std::vector<float> &objBPos, std::vector<float> &objAsize) {
    if (objAPos[0] <= objBPos[0] && objAPos[0] + objAsize[0] >= objBPos[0]) {
        if (objAPos[1] <= objBPos[1] && objAPos[1] + objAsize[1] >= objBPos[1]) {
            if (objAPos[2] <= objBPos[2] && objAPos[2] + objAsize[2] >= objBPos[2]) {
                return true;
            }
        }
    }
    return false;
}

bool aabbccColisoion(std::vector<float> &objAPos, std::vector<float> &objBPos, std::vector<float> &objAsize, std::vector<float> &objBsize) {
    if (objAPos[0] <= objBPos[0] && objAPos[0] + objAsize[0] >= objBPos[0]) {
        if (objAPos[1] <= objBPos[1] && objAPos[1] + objAsize[1] >= objBPos[1]) {
            if (objAPos[2] <= objBPos[2] && objAPos[2] + objAsize[2] >= objBPos[2]) {
                return true;
            }
            if (objAPos[2] <= objBPos[2] + objBsize[2] && objAPos[2] + objAsize[2] >= objBPos[2] + objBsize[2]) {
                return true;
            }
        }
        if (objAPos[1] <= objBPos[1] + objBsize[1] && objAPos[1] + objAsize[1] >= objBPos[1] + objBsize[1]) {
            if (objAPos[2] <= objBPos[2] && objAPos[2] + objAsize[2] >= objBPos[2]) {
                return true;
            }
            if (objAPos[2] <= objBPos[2] + objBsize[2] && objAPos[2] + objAsize[2] >= objBPos[2] + objBsize[2]) {
                return true;
            }
        }
    }
    if (objAPos[0] <= objBPos[0] + objBsize[0] && objAPos[0] + objAsize[0] >= objBPos[0] + objBsize[0]) {
        if (objAPos[1] <= objBPos[1] && objAPos[1] + objAsize[1] >= objBPos[1]) {
            if (objAPos[2] <= objBPos[2] && objAPos[2] + objAsize[2] >= objBPos[2]) {
                return true;
            }
            if (objAPos[2] <= objBPos[2] + objBsize[2] && objAPos[2] + objAsize[2] >= objBPos[2] + objBsize[2]) {
                return true;
            }
        }
        if (objAPos[1] <= objBPos[1] + objBsize[1] && objAPos[1] + objAsize[1] >= objBPos[1] + objBsize[1]) {
            if (objAPos[2] <= objBPos[2] && objAPos[2] + objAsize[2] >= objBPos[2]) {
                return true;
            }
            if (objAPos[2] <= objBPos[2] + objBsize[2] && objAPos[2] + objAsize[2] >= objBPos[2] + objBsize[2]) {
                return true;
            }
        }
    }
    return false;
}


uint32_t converColor(int red, int green, int blue) {
    uint32_t finalColor = 0xFF000000;
    finalColor |= red << 16;
    finalColor |= green << 8;
    finalColor |= blue;
    return finalColor;
}

void quickAbsValueInt(int &numberA) {
    if (numberA < 0) {
        numberA *= -1;
    }
}

bool aproximation(float numberA, float numberB, float depth) {
    if (depth >= numberA - numberB && -depth <= numberA - numberB) {
        return true;
    }
    return false;
}

float dotProductFloat(std::vector<float> &vecA, std::vector<float> &vecB) {
    if (vecA.size() == vecB.size()) {
        float sum = 0;
        for (int i = 0; i < vecA.size(); i += 1) { // size je vzdy o 1 vetsi nez max index
            sum += vecA[i]*vecB[i];
        }
        return sum;
    }
    std::cout << "To je v prdeli, tvoje vektory maji jine dimenze" << std::endl;
    return -1;
}

float crossProductFloat2D(std::vector<float> &vecA, std::vector<float> &vecB) {
    return vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

std::vector<int> findMinsAndMax2D(std::vector<std::vector<int>> &listPoints) {
    int maxX = listPoints[0][0];
    int maxY = listPoints[0][1];
    int minX = listPoints[0][0];
    int minY = listPoints[0][1];
    for (std::vector<int> onePoint : listPoints) {
        if (onePoint[0] >= maxX) {
            maxX = onePoint[0];
        }
        if (onePoint[1] >= maxY) {
            maxY = onePoint[1];
        }
        if (onePoint[0] <= minX) {
            minX = onePoint[0];
        }
        if (onePoint[1] <= minY) {
            minY = onePoint[1];
        }
    }
    return {maxX,maxY,minX,minY};
}

std::vector<float> makeVector(std::vector<float> &posA, std::vector<float> &posB) {
    if (posA.size() == posB.size()) {
        std::vector<float> returnVec = {};
        for (int i = 0; i < posA.size(); i += 1) {
            returnVec.push_back(posA[i] - posB[i]);
        }
        return returnVec;
    }
    std::cout << "To je v prdeli, tvoje body maji jine dimenze" << std::endl;
    return {-1,-1,-1};
}

std::vector<float> makeVectorInt(std::vector<int> posA, std::vector<int> &posB) {
    if (posA.size() == posB.size()) {
        std::vector<float> returnVec = {};
        for (int i = 0; i < posA.size(); i += 1) {
            returnVec.push_back(posA[i] - posB[i]);
        }
        return returnVec;
    }
    std::cout << "To je v prdeli, tvoje body maji jine dimenze" << std::endl;
    return {-1,-1,-1};
}

std::vector<int> makeIntoScreenCors(std::vector<float> pointPos, int fov) {
    if (pointPos[2] > 0.1f) {
        float radiansFOV = fov * (M_PI / 180);
        float numberAmpX = ((MonitorWidth / 2) / (std::tan(radiansFOV/2)));
        float numberAmpY = ((MonitorHeight / 2) / (std::tan(radiansFOV/2)));
        pointPos[0] /= pointPos[2];
        pointPos[1] /= pointPos[2]; // cim dal tim je to bliz ke stredu protoze ty 2 kolmice ven expanduje do nekonecna
        return {int(std::round((pointPos[0] * numberAmpX) + (MonitorWidth/2))), int(std::round((pointPos[1] * -numberAmpY) + (MonitorHeight/2)))};
    }
    std::cout << "To je v prdeli, to co chces vykreslit tak je za kamerou" << std::endl;
    return {-1,-1};
}


void drawPixel(int pitch, uint32_t* pixelBuffer, int xPos, int yPos, int red, int green, int blue) {
    if (xPos > 0 && xPos < MonitorWidth && yPos > 0 && yPos < MonitorHeight) {
        pixelBuffer[yPos * (pitch / 4) + xPos] = converColor(red, green, blue);
    }
}


void drawSingleLine(std::vector<int> position1, std::vector<int> position2, std::vector<int> &color, SDL_Texture* buffer, int pitch, uint32_t* pixelBuffer) {
    std::vector<int> helpfulThings = thingsForLine(position1, position2);

    for (int i = 0; i < helpfulThings[2]; i += 1) {
        drawPixel(pitch, pixelBuffer, int(std::round(position1[0] + helpfulThings[0] * i)), int(std::round(position1[1] + helpfulThings[1] * i)), color[0], color[1], color[2]);
    }
}


void drawPolygon(std::vector<int> &positionLeftUp, std::vector<int> &positionRightUp, std::vector<int> &positionLeftDown, std::vector<int> &positionRightDown, SDL_Texture* buffer, int pitch, uint32_t* pixelBuffer, bool includeBorder, std::vector<int> &color) {

    std::vector<std::vector<int>> positions = {positionLeftUp,positionRightUp,positionLeftDown,positionRightDown};
    std::vector<int> minsAmaxs = findMinsAndMax2D(positions);

    for (int yPos = minsAmaxs[3]; yPos < minsAmaxs[1]; yPos += 1) {
        for (int xPos = minsAmaxs[2]; xPos < minsAmaxs[0]; xPos += 1) {

            std::vector<float> vectorSA = makeVectorInt({xPos,yPos},positionLeftUp);
            std::vector<float> vectorSB = makeVectorInt({xPos,yPos},positionRightUp);
            if (crossProductFloat2D(vectorSA,vectorSB) < 0) {
                continue;
            }
            std::vector<float> vectorSC = makeVectorInt({xPos,yPos},positionRightDown);
            if (crossProductFloat2D(vectorSB,vectorSC) < 0) {
                continue;
            }
            std::vector<float> vectorSD = makeVectorInt({xPos,yPos},positionLeftDown);
            if (crossProductFloat2D(vectorSC,vectorSD) < 0) {
                continue;
            }
            if (crossProductFloat2D(vectorSD,vectorSA) < 0) {
                continue;
            }

            drawPixel(pitch, pixelBuffer, xPos, yPos, color[0], color[1], color[2]);
        }
    }

}


struct Point{
    float x,y,z;
    std::vector<int> color;
    std::vector<bool> grown;
    Point(std::vector<float> position, std::vector<int> color, std::vector<bool> grown) {
        this->x = position[0];
        this->y = position[1];
        this->z = position[2];
        this->color = color;
        this->grown = grown;
    }
    std::vector<float> getRelativePos(std::vector<float> &headingVec, std::vector<float> &headingOrigin, float angleY, float angleZ) { // x, y - na obrazovce (vyska a sirka) a z je potom hloubka
        std::vector<float> myPos = {x,y,z};
        std::vector<float> upVector = {std::cos(angleY)*std::cos(angleZ + float(M_PI / 2)),std::sin(angleY)*std::cos(angleZ + float(M_PI / 2)),std::sin(angleZ + float(M_PI / 2))};
        std::vector<float> rightVector = {std::cos(angleY + float(M_PI / 2))*std::cos(angleZ),std::sin(angleY+float(M_PI / 2))*std::cos(angleZ),std::sin(angleZ)};
        std::vector<float> vectorOS = makeVector(myPos,headingOrigin);
        float yPos = dotProductFloat(vectorOS,upVector);
        float xPos = dotProductFloat(vectorOS,rightVector);
        float distanceSmall = dotProductFloat(headingVec,vectorOS);
        return {xPos, yPos, distanceSmall};
    }
};


struct Cube {
    float angle_z, angle_y;
    float xPos, yPos, zPos;
    float sizeX, sizeY, sizeZ;
    std::vector<Point> importantPoints = {};
    std::vector<int> color;
    std::vector<std::vector<int>> pointLinks;
    bool stroked;
    Cube(std::vector<float> position, std::vector<float> rotation, std::vector<float> sizes, std::vector<int> color, bool stroked) {
        this->stroked = stroked;
        this->xPos = position[0];
        this->yPos = position[1];
        this->zPos = position[2];
        this->sizeX = sizes[0];
        this->sizeY = sizes[1];
        this->sizeZ = sizes[2];
        this->angle_y = rotation[0];
        this->angle_z = rotation[1];
        this->color = color;
        this->importantPoints = {
            Point({xPos,yPos,zPos}, color, {false,false,false}), Point({xPos + sizeX,yPos,zPos}, color, {true,false,false}), Point({xPos + sizeX,yPos + sizeY,zPos}, color, {true,true,false}),
            Point({xPos + sizeX,yPos + sizeY,zPos + sizeZ}, color, {true,true,true}),Point({xPos, yPos + sizeY,zPos + sizeZ}, color, {false,true,true}),
            Point({xPos, yPos, zPos + sizeZ}, color, {false,false,true}), Point({xPos, yPos + sizeY,zPos}, color, {false,true,false}), Point({xPos + sizeX, yPos,zPos + sizeZ}, color, {true,false,true})};
        this->pointLinks = {{0,1,2,6}, {3,4,5,7}, {0,5,4,6}, {1,2,3,7}, {0,1,7,5}, {2,3,4,6}}; // manually calibrated for points in order for every face of the square
    }

    void update(std::vector<float> movement, std::vector<float> growth, std::vector<float> rotation) {
        xPos += movement[0];
        yPos += movement[1];
        zPos += movement[2];
        sizeX += growth[0];
        sizeY += growth[1];
        sizeZ += growth[2];
        angle_y += rotation[0];
        angle_z += rotation[1];
        for (Point onePoint : importantPoints) {
            onePoint.x = xPos;
            onePoint.y = yPos;
            onePoint.z = zPos;
            if (onePoint.grown[0]) {
                onePoint.x += growth[0];
            }
            if (onePoint.grown[1]) {
                onePoint.y += growth[1];
            }
            if (onePoint.grown[2]) {
                onePoint.z += growth[2];
            }
        }
    }
};


struct Player {
    float angle_z, angle_y;
    int fov;
    float speed;
    std::vector<float> headingVec = {0,0,0};
    std::vector<float> myPos = {0,0,0};
    Player(std::vector<float> position, int fov, float speed) {
        this->myPos = position;
        this->fov = fov;
        this->speed = speed;
        this->angle_y = 0;
        this->angle_z = 0;
        this->headingVec = {std::cos(angle_y)*std::cos(angle_z),std::sin(angle_y)*std::cos(angle_z),std::sin(angle_z)};
    }
    void camera(std::vector<Cube> objects, SDL_Texture* buffer, uint32_t* pixelBuffer, int pitch) {
        for (Cube oneObject : objects) {
            std::vector<std::vector<float>> relativePoses = {};
            for (Point onePoint : oneObject.importantPoints) {
                relativePoses.push_back(onePoint.getRelativePos(headingVec,myPos,angle_y,angle_z));
            }
            if (oneObject.stroked) {
                for (std::vector<int> onePointPos : oneObject.pointLinks) {
                    drawSingleLine(makeIntoScreenCors(relativePoses[onePointPos[0]], fov), makeIntoScreenCors(relativePoses[onePointPos[1]], fov), oneObject.color, buffer, pitch, pixelBuffer);
                    drawSingleLine(makeIntoScreenCors(relativePoses[onePointPos[1]], fov), makeIntoScreenCors(relativePoses[onePointPos[2]], fov), oneObject.color, buffer, pitch, pixelBuffer);
                    drawSingleLine(makeIntoScreenCors(relativePoses[onePointPos[2]], fov), makeIntoScreenCors(relativePoses[onePointPos[3]], fov), oneObject.color, buffer, pitch, pixelBuffer);
                    drawSingleLine(makeIntoScreenCors(relativePoses[onePointPos[3]], fov), makeIntoScreenCors(relativePoses[onePointPos[0]], fov), oneObject.color, buffer, pitch, pixelBuffer);
                }
            }
        }
    }
    void movement() {
        //std::cout << "Hrac pozice (x,y,z): " << xPos << yPos << zPos << "hrac rotace y-rovina: " << angle_y << "z-rovina: " << angle_z << std::endl;
        if (state[SDL_SCANCODE_W]) {
            myPos[0] += speed*std::cosf(angle_y);
            myPos[1] += speed*std::sinf(angle_y);
        }
        if (state[SDL_SCANCODE_S]) {
            myPos[0] -= speed*std::cosf(angle_y);
            myPos[1] -= speed*std::sinf(angle_y);
        }
        if (state[SDL_SCANCODE_D]) {
            myPos[0] -= speed*std::cosf(angle_y - M_PI/2);
            myPos[1] -= speed*std::sinf(angle_y - M_PI/2);
        }
        if (state[SDL_SCANCODE_A]) {
            myPos[0] += speed*std::cosf(angle_y - M_PI/2);
            myPos[1] += speed*std::sinf(angle_y - M_PI/2);
        }
        if (state[SDL_SCANCODE_SPACE]) {
            myPos[2] -= speed;
        }
        if (state[SDL_SCANCODE_LSHIFT]) {
            myPos[2] += speed;
        }
        if (state[SDL_SCANCODE_UP]) {
            if (angle_z >= -1) {
                angle_z -= 0.05;
                headingVec = {std::cos(angle_y)*std::cos(angle_z),std::sin(angle_y)*std::cos(angle_z),std::sin(angle_z)};
            }
        }
        if (state[SDL_SCANCODE_DOWN]) {
            if (angle_z <= 1) {
                angle_z += 0.05;
                headingVec = {std::cos(angle_y)*std::cos(angle_z),std::sin(angle_y)*std::cos(angle_z),std::sin(angle_z)};
            }
        }
        if (state[SDL_SCANCODE_LEFT]) {
            angle_y -= 0.05;
            headingVec = {std::cos(angle_y)*std::cos(angle_z),std::sin(angle_y)*std::cos(angle_z),std::sin(angle_z)};
        }
        if (state[SDL_SCANCODE_RIGHT]) {
            angle_y += 0.05;
            headingVec = {std::cos(angle_y)*std::cos(angle_z),std::sin(angle_y)*std::cos(angle_z),std::sin(angle_z)};
        }
    }
};



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
    std::vector<Cube> naseObjekty = {};

    naseObjekty.push_back(Cube({10,0,0},{0,0},{5,2,2},{0,0,0},true));
    naseObjekty.push_back(Cube({-10,0,0},{0,0},{5,2,2},{250,0,0},true));
    naseObjekty.push_back(Cube({0,10,0},{0,0},{5,2,2},{250,0,0},true));
    naseObjekty.push_back(Cube({0,-10,0},{0,0},{5,2,2},{250,0,0},true));
    naseObjekty.push_back(Cube({0,0,10},{0,0},{5,2,2},{250,0,0},true));
    naseObjekty.push_back(Cube({0,0,-10},{0,0},{5,2,2},{250,0,0},true));

    uint32_t* nasPixelBuffer;
    int pitch;

    Player hrac = Player({0,0,0},120,0.1);

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
        SDL_SetRenderDrawColor(renderer, 0, 50, 250, 255);
        SDL_RenderClear(renderer);
        SDL_LockTexture(buffer, NULL, (void**)&nasPixelBuffer, &pitch);
        std::fill(nasPixelBuffer, nasPixelBuffer + MonitorHeight * MonitorWidth, 0xff0000ff);
        hrac.camera(naseObjekty, buffer, nasPixelBuffer, pitch);
        SDL_UnlockTexture(buffer);
        SDL_RenderCopy(renderer, buffer, NULL, NULL);
        hrac.movement();
        SDL_RenderPresent(renderer);
    }


    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();


    return 0;
}
