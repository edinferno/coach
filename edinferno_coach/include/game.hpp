#ifndef GAME_HPP
#define GAME_HPP


enum GameStrategy {Defence, Attack, KeepCalmAndCarryOn};

class Game
{
public:
	static bool BroadcastStrategy(GameStrategy strategy);
};
#endif
