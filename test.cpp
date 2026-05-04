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
#include "./main.cpp"


int main(int argc, char* argv[]) {
    bool running = true;

    gameInfo game = gameInfo(1980, 1080, "Super hra");

    Player_Double myPlayer = createBasicPlayer(game, 90, 0.005, 0.8);

    for (int i = 0; i < 30; i += 1) {
        createBasicCube(game, myPlayer, simple3D_Pos_Double(getRandomDouble(-100,100), getRandomDouble(-100,100), getRandomDouble(-100,100)),
            simple3D_Pos_Double(getRandomDouble(2,10), getRandomDouble(2,10), getRandomDouble(2,10)),
            SimpleColor(getRandomInt(0,100), getRandomInt(0,100), getRandomInt(0,100)), SimpleColor(0,0,0), true, 10, false);
    }

    while (running) {
        if (game.turnOffButton()) {
            running = false;
        }
        if (game.isKeyPressed(SDLK_ESCAPE)) {
            running = false;
        }
        moveBasicCube(game, 0, myPlayer, simple3D_Pos_Double(0.02,0,0));

        game.playerCameraMovement(myPlayer);
        game.playerMovement(myPlayer);
        game.drawScene(myPlayer);
    }

    return 0;
}