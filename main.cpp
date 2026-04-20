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


struct simple3D_Pos_Double {
    double x, y, z;

    simple3D_Pos_Double(double x = 0, double y = 0, double z = 0) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};



class Vector3D_Double {
private:
    double absoluteLenght;
    simple3D_Pos_Double myPos;

public:

    Vector3D_Double(simple3D_Pos_Double const &impPos) {
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
        simple3D_Pos_Double quickPos = secondVec.getVecRef();
        return (quickPos.x * myPos.x) + (quickPos.y * myPos.y) + (quickPos.z * myPos.z);
    }

    void setVector(simple3D_Pos_Double const &impPos) {
        myPos.x = impPos.x;
        myPos.y = impPos.y;
        myPos.z = impPos.z;
        absoluteLenght = std::sqrt((myPos.x * myPos.x) + (myPos.y * myPos.y) + (myPos.z * myPos.z));
    }

    double absoluteValue() {
        return absoluteLenght;
    }

    double crossProduct2D(Vector3D_Double &secondVec) {
        simple3D_Pos_Double quickPos = secondVec.getVecRef();
        return myPos.x * quickPos.y - myPos.y * quickPos.x;
    }
};




struct Vector3D {
    double x,y,z;

    Vector3D(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    static int size() {
        return 3;
    }

    double dotProduct(Vector3D const &secondVec) {
        return (secondVec.x * x) + (secondVec.y * y) + (secondVec.z * z);
    }

    double absoluteValue() {
        return std::sqrt((x * x) + (y * y) + (z * z));
    }

    double crossProduct2D(Vector3D const &secondVec) {
        return x * secondVec.y - y * secondVec.x;
    }

    Vector3D crossProduct3D(Vector3D const &secondVec) {
        return Vector3D((z * secondVec.y) - (y * secondVec.z),  (z * secondVec.x) - (x * secondVec.z), (x * secondVec.y) - (y * secondVec.x));
    }
};


struct SimpleColor {
    int red, blue, green, transp;
    SimpleColor(int red, int blue, int green, int transp) {
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


struct ZBuffer {
    double zPos;
    ZBuffer(double impZPos) {
        this->zPos = impZPos;
    }
};


struct Pseudo3DColor {
    double z;
    SimpleColor color = SimpleColor(0,0,0,0);
    
    Pseudo3DColor(double z, SimpleColor impCol) {
        this->color.red = impCol.red;
        this->color.blue = impCol.blue;
        this->color.green = impCol.green;
        this->color.transp = impCol.transp;
        this->z = z;
    }

    uint32_t convertToBinary() {
        uint32_t finalColor = 0x00000000;
        finalColor |= color.transp << 24;
        finalColor |= color.red << 16;
        finalColor |= color.green << 8;
        finalColor |= color.blue;
        return finalColor;
    }
};


struct Position3D {
    double x,y,z;

    Position3D(double x = 0, double y = 0, double z = 0) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    static int size() {
        return 3;
    }

    Vector3D makeAVector(Position3D const &secondPos) {
        return Vector3D(secondPos.x -x,secondPos.y - y,secondPos.z - z);
    }

    Vector3D makeA2DVector(Position3D const &secondPos) {
        return Vector3D(x - secondPos.x, y - secondPos.y, 0);
    }


    Position3D changedBy(double impX, double impY, double impZ) {
        return Position3D(x + impX, y + impY, z + impZ);
    }

    Position3D makeIntoScreensCord(int fov) {
        if (z > 0.1) {
            double radiansFOV = fov * (M_PI / 180);
            double numberAmpX = ((MonitorWidth / 2) / (std::tan(radiansFOV/2)));
            double numberAmpY = ((MonitorHeight / 2) / (std::tan(radiansFOV/2)));
            return Position3D(std::round(((x/z) * numberAmpX) + (MonitorWidth/2)), std::round(((y/z) * numberAmpY) + (MonitorHeight/2)), z);
        }
        return Position3D(-1, -1, -10);
    }
};


std::array<double, 2> getGradiants(Position3D &pointA, Position3D &pointB, Position3D &pointC) {
    double diffX1 = pointB.x - pointA.x;
    double diffX2 = pointC.x - pointB.x;
    double diffY1 = pointB.y - pointA.y;
    double diffY2 = pointC.y - pointB.y;
    double diffZ1 = pointB.z - pointA.z;
    double diffZ2 = pointC.z - pointA.z;
    double determinant = diffX1 * diffY2 - diffX2 * diffY1;
    if (std::abs(determinant) <= 0.01) {
        std::cout << "spatny gradiant jedna cara" << std::endl;
        determinant = 0.02;
    }
    double gradiantX = (diffY2 * diffZ1 - diffZ2 * diffY1) / (determinant);
    double gradiantY = (diffX1 * diffZ2 - diffZ1 * diffX2) / (determinant);
    return {gradiantX, gradiantY};
}


// trojuhelnik
struct ScreenPolygon {
    std::array<Position3D, 3> points = {};

    ScreenPolygon(Position3D point1, Position3D point2, Position3D point3) {
        points[0] = point1;
        points[1] = point2;
        points[2] = point3;
    }

    std::array<int, 4> minsAndMaxs(int screenWidth,int screenHeight) {
        std::array<int, 4> minsAMax = {int(points[0].x), int(points[0].y), int(points[0].x), int(points[0].y)};
        for (int i = 0; i < 3; i += 1) {
            if (points[i].x >= minsAMax[0]) {
                minsAMax[0] = int(points[i].x);
            }
            if (points[i].y >= minsAMax[1]) {
                minsAMax[1] = int(points[i].y);
            }

            if (points[i].x <= minsAMax[2]) {
                minsAMax[2] = int(points[i].x);
            }
            if (points[i].y <= minsAMax[3]) {
                minsAMax[3] = int(points[i].y);
            }
        }
        //border pripady

        if (minsAMax[2] <= 0) {
            minsAMax[2] = 0;
        }
        if (minsAMax[3] <= 0) {
            minsAMax[3] = 0;
        }
        if (minsAMax[0] >= screenWidth) {
            minsAMax[0] = screenWidth;
        }
        if (minsAMax[1] >= screenHeight) {
            minsAMax[1] = screenHeight;
        }
        return minsAMax;
    }


    void drawOutPolygon(Pseudo3DColor* colorsBuffer, int screenWidth,int screenHeight, SimpleColor polygonCol, bool outLine) {
        std::array<int, 4> minsAmaxs = minsAndMaxs(screenWidth, screenHeight);
        std::array<double, 2> gradiant = getGradiants(points[0], points[1], points[2]);
        double outLineBoundary = (minsAmaxs[0] - minsAmaxs[2]) * (minsAmaxs[1] - minsAmaxs[3]) * 0.01;

        for (int yPos = minsAmaxs[3]; yPos < minsAmaxs[1]; yPos += 1) {
            for (int xPos = minsAmaxs[2]; xPos < minsAmaxs[0]; xPos += 1) {

                Position3D myPos = Position3D(xPos, yPos);
                SimpleColor pixelColor = polygonCol;

                Vector3D vectorSA = points[0].makeA2DVector(myPos);
                Vector3D vectorSB = points[1].makeA2DVector(myPos);
                double abCrossProd = vectorSA.crossProduct2D(vectorSB);

                if (abCrossProd < 0) {
                    continue;
                }

                Vector3D vectorSC = points[2].makeA2DVector(myPos);
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
                        pixelColor.blue = 0;
                        pixelColor.green = 0;
                        pixelColor.red = 0;
                    }
                }

                double globalZ = points[2].z + (xPos -points[2].x) * gradiant[0] + (yPos - points[2].y) * gradiant[1];
                if (globalZ < colorsBuffer[xPos + (yPos * screenWidth)].z ) {
                    colorsBuffer[xPos + (yPos * screenWidth)].color = pixelColor;
                    colorsBuffer[xPos + (yPos * screenWidth)].z = globalZ;
                }
            }
        }

    }
};


struct Point{
    Position3D position = Position3D(0,0,0);
    SimpleColor color = SimpleColor(0,0,0,0);
    std::vector<bool> grown;

    Point(Position3D impPosition, SimpleColor color, std::vector<bool> grown) {
        this->color = color;
        this->grown = grown;
        this->position.x = impPosition.x;
        this->position.y = impPosition.y;
        this->position.z = impPosition.z;
    }

    Position3D getRelativePos(Vector3D &headingVec, Position3D &headingOrigin, double angleY, double angleZ) { // x, y - na obrazovce (vyska a sirka) a z je potom hloubka

        Vector3D upVector = Vector3D(std::cos(angleY)*std::cos(angleZ - M_PI / 2), std::sin(angleY) * std::cos(angleZ - M_PI / 2),  std::sin(angleZ - M_PI / 2));
        Vector3D rightVector = Vector3D(std::cos(angleY + M_PI / 2) * std::cos(angleZ), std::sin(angleY + M_PI / 2) * std::cos(angleZ), 0);
        Vector3D vectorOS = headingOrigin.makeAVector(position);

        return Position3D(vectorOS.dotProduct(rightVector), vectorOS.dotProduct(upVector), vectorOS.dotProduct(headingVec));
    }
};


struct Cube {
    double angle_z, angle_y;
    Position3D position = Position3D(0,0,0);
    Position3D size = Position3D(0,0,0);
    SimpleColor color = SimpleColor(0,0,0,0);

    std::array<Point, 8> importantPoints = {
        Point(position, color, {false,false,false}), Point(position.changedBy(size.x, 0, 0), color, {true,false,false}),
        Point(position.changedBy(size.x, size.y, 0), color, {true,true,false}),
        Point(position.changedBy(size.x, size.y, size.z), color, {true,true,true}),Point(position.changedBy(0, size.y, size.z), color, {false,true,true}),
        Point(position.changedBy(0, 0, size.z), color, {false,false,true}), Point(position.changedBy(0, size.y, 0), color, {false,true,false}),
        Point(position.changedBy(size.x, 0, size.z), color, {true,false,true})
    };;

    using Triangle = std::array<int, 3>;
    // tohle je jedina vec co jsem vygeneroval pres AI na to abych nemusel manualne delat kazdy trojuhelnik
    std::array<std::array<int, 3>, 12> pointLinks = {
        Triangle{0,2,1}, Triangle{0,6,2},  // Z=0
        Triangle{3,4,5}, Triangle{3,5,7},  // Z=1
        Triangle{0,5,4}, Triangle{0,4,6},  // X=0
        Triangle{1,2,3}, Triangle{1,3,7},  // X=1
        Triangle{0,1,7}, Triangle{0,7,5},  // Y=0
        Triangle{2,4,3}, Triangle{2,6,4}   // Y=1
    };
    bool stroked;

    Cube(Position3D impPosition, std::vector<double> rotation, Position3D impSizes, SimpleColor color, bool stroked) {
        this->stroked = stroked;

        this->position.x = impPosition.x;
        this->position.y = impPosition.y;
        this->position.z = impPosition.z;

        this->size.x = impSizes.x;
        this->size.y = impSizes.y;
        this->size.z = impSizes.z;

        this->angle_y = rotation[0];
        this->angle_z = rotation[1];

        this->color = color;


        this->importantPoints[0] = Point(position, color, {false,false,false});
        this->importantPoints[1] = Point(position.changedBy(size.x, 0, 0), color, {true,false,false});
        this->importantPoints[2] = Point(position.changedBy(size.x, size.y, 0), color, {true,true,false});
        this->importantPoints[3] = Point(position.changedBy(size.x, size.y, size.z), color, {true,true,true});
        this->importantPoints[4] = Point(position.changedBy(0, size.y, size.z), color, {false,true,true});
        this->importantPoints[5] = Point(position.changedBy(0, 0, size.z), color, {false,false,true});
        this->importantPoints[6] = Point(position.changedBy(0, size.y, 0), color, {false,true,false});
        this->importantPoints[7] = Point(position.changedBy(size.x, 0, size.z), color, {true,false,true});
    }

    void update(std::vector<double> movement, std::vector<double> growth, std::vector<double> rotation) {
        position.x += movement[0];
        position.y += movement[1];
        position.z += movement[2];

        size.x += growth[0];
        size.y += growth[1];
        size.z += growth[2];

        angle_y += rotation[0];
        angle_z += rotation[1];

        for (Point &onePoint : importantPoints) {
            onePoint.position.x = position.x;
            onePoint.position.y = position.y;
            onePoint.position.z = position.z;

            if (onePoint.grown[0]) {
                onePoint.position.x += size.x;
            }
            if (onePoint.grown[1]) {
                onePoint.position.y += size.y;
            }
            if (onePoint.grown[2]) {
                onePoint.position.z += size.z;
            }

        }
    }

    void drawOut(Vector3D &headingVec, Position3D &headingOrigin, double angleY, double angleZ, int screenWidth, int screenHeight, Pseudo3DColor* colorsBuffer, int fov) {
        std::array<Position3D, 8> pseudoPoints = {};
        for (int i = 0; i < 8; i += 1) {
            pseudoPoints[i] = (importantPoints[i].getRelativePos(headingVec, headingOrigin, angleY, angleZ)).makeIntoScreensCord(fov);
        }
        for (int i = 0; i < 12; i += 1) {
            if (pseudoPoints[pointLinks[i][0]].z < 0.1 || pseudoPoints[pointLinks[i][1]].z < 0.1 || pseudoPoints[pointLinks[i][2]].z < 0.1) {
                continue;
            }

            ScreenPolygon onePolygon = ScreenPolygon(pseudoPoints[pointLinks[i][0]], pseudoPoints[pointLinks[i][1]], pseudoPoints[pointLinks[i][2]]);
            onePolygon.drawOutPolygon(colorsBuffer, screenWidth, screenHeight, color, stroked);
        }
    }
};


struct Player {
    double angle_z, angle_y;
    int fov;
    double speed;
    Vector3D headingVec = Vector3D(0,0,0);
    Position3D myPos = Position3D(0,0,0);
    Player(Position3D impPosition, int fov, double speed) {
        this->myPos.x = impPosition.x;
        this->myPos.y = impPosition.y;
        this->myPos.z = impPosition.z;
        this->fov = fov;
        this->speed = speed;
        this->angle_y = 0;
        this->angle_z = 0;
        this->headingVec.x = std::cos(angle_y) * std::cos(angle_z);
        this->headingVec.y = std::sin(angle_y) * std::cos(angle_z);
        this->headingVec.z = std::sin(angle_z);
    }

    void camera(std::vector<Cube> const &objects, Pseudo3DColor* colorsBuffer ,int screenWidth, int screenHeight) {
        for (Cube oneObject : objects) {
            oneObject.drawOut(headingVec, myPos, angle_y, angle_z, screenWidth, screenHeight, colorsBuffer, fov);
        }
    }

    void movement() {
        //std::cout << "Hrac pozice (x,y,z): " << xPos << yPos << zPos << "hrac rotace y-rovina: " << angle_y << "z-rovina: " << angle_z << std::endl;
        if (state[SDL_SCANCODE_W]) {
            myPos.x += speed*std::cosf(angle_y);
            myPos.y += speed*std::sinf(angle_y);
        }
        if (state[SDL_SCANCODE_S]) {
            myPos.x -= speed*std::cosf(angle_y);
            myPos.y -= speed*std::sinf(angle_y);
        }
        if (state[SDL_SCANCODE_D]) {
            myPos.x -= speed*std::cosf(angle_y - M_PI/2);
            myPos.y -= speed*std::sinf(angle_y - M_PI/2);
        }
        if (state[SDL_SCANCODE_A]) {
            myPos.x += speed*std::cosf(angle_y - M_PI/2);
            myPos.y += speed*std::sinf(angle_y - M_PI/2);
        }
        if (state[SDL_SCANCODE_SPACE]) {
            myPos.z += speed;
        }
        if (state[SDL_SCANCODE_LSHIFT]) {
            myPos.z -= speed;
        }
        if (state[SDL_SCANCODE_DOWN]) {
            if (angle_z >= -1) {
                angle_z -= 0.05;
                headingVec.x = std::cos(angle_y) * std::cos(angle_z);
                headingVec.y = std::sin(angle_y) * std::cos(angle_z);
                headingVec.z = std::sin(angle_z);
            }
        }
        if (state[SDL_SCANCODE_UP]) {
            if (angle_z <= 1) {
                angle_z += 0.05;
                headingVec.x = std::cos(angle_y) * std::cos(angle_z);
                headingVec.y = std::sin(angle_y) * std::cos(angle_z);
                headingVec.z = std::sin(angle_z);
            }
        }
        if (state[SDL_SCANCODE_LEFT]) {
            angle_y -= 0.05;
            headingVec.x = std::cos(angle_y) * std::cos(angle_z);
            headingVec.y = std::sin(angle_y) * std::cos(angle_z);
            headingVec.z = std::sin(angle_z);
        }
        if (state[SDL_SCANCODE_RIGHT]) {
            angle_y += 0.05;
            headingVec.x = std::cos(angle_y) * std::cos(angle_z);
            headingVec.y = std::sin(angle_y) * std::cos(angle_z);
            headingVec.z = std::sin(angle_z);
        }
    }
};



void changeToSDL2(Pseudo3DColor* myColorBuffer, uint32_t* pixelBuffer, int pitch, int screenHeight, int screenWidth) {
    int constantPitchPixel = pitch / 4;

    for (int yPos = 0; yPos < screenHeight; yPos += 1) {
        Pseudo3DColor* srcRow = myColorBuffer + (yPos * screenWidth);
        uint32_t* sld2Textur = pixelBuffer + (yPos * constantPitchPixel);

        for (int xPos = 0; xPos < screenWidth; xPos += 1) {
            sld2Textur[xPos] = srcRow[xPos].convertToBinary();
        }
    }
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
    std::vector<Cube> naseObjekty = {};
    uint32_t* nasPixelBuffer;
    Pseudo3DColor* myColorBuffer = nullptr;
    myColorBuffer = (Pseudo3DColor*)malloc(MonitorHeight * MonitorWidth * sizeof(Pseudo3DColor));
    std::fill(myColorBuffer, myColorBuffer + MonitorHeight * MonitorWidth, Pseudo3DColor(1000.0, SimpleColor(0, 255, 0, 255)));
    int pitch;
    naseObjekty.push_back(Cube(Position3D(4,-5,1), {0,0}, Position3D(3,5,4), SimpleColor(20,100,255,255), true));
    naseObjekty.push_back(Cube(Position3D(4,15,1), {0,0}, Position3D(3,5,4), SimpleColor(255,0,0,255), false));
    naseObjekty.push_back(Cube(Position3D(4,12,10), {0,0}, Position3D(3,5,4), SimpleColor(255,250,0,255), true));
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

        SDL_RenderClear(renderer);
        SDL_LockTexture(buffer, NULL, (void**)&nasPixelBuffer, &pitch);
        std::fill(nasPixelBuffer, nasPixelBuffer + MonitorHeight * MonitorWidth, 0xff0000ff);
        std::fill(myColorBuffer, myColorBuffer + MonitorHeight * MonitorWidth, Pseudo3DColor(1000.0, SimpleColor(0, 255, 0, 255)));

        hrac.camera(naseObjekty, myColorBuffer, MonitorWidth, MonitorHeight);
        changeToSDL2(myColorBuffer, nasPixelBuffer, pitch, MonitorHeight, MonitorWidth);

        SDL_UnlockTexture(buffer);
        SDL_RenderCopy(renderer, buffer, NULL, NULL);
        hrac.movement();


        SDL_RenderPresent(renderer);
    }


    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::free(myColorBuffer);


    return 0;
}
