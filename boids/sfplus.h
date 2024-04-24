#pragma once

#include <SFML/Graphics.hpp>
#include "sfvec.h"

#include <iostream>
#include <functional>
#include <vector>
#include <string>

//! incomplete class for GUI selection of execution mode, does not affect simulation and is unused

class Button : public sf::Sprite {
private:
	std::string text;

	bool state;

	sf::Texture normalTexture;
	sf::Texture pressedTexture;

	// TODO add error handling for actions (i.e. returning >0 for successful and <0 for unsuccessful
	std::function<void(bool)> action;
public:
	Button(std::string text, sf::Vector2f position = sfvec::ZEROF) {
		if (!this->normalTexture.loadFromFile("./button.png") || !this->pressedTexture.loadFromFile("./button_pressed.png")) {
			std::cerr << "ERROR: ./button.png OR ./button_pressed.png could not be found!" << std::endl;
		}

		this->setTexture(normalTexture);
		this->setPosition(position);

		this->text = text;
		this->state = false;
	}

	template<typename F>
	void bind(F callback) {
		this->action = callback;
	}

	void trigger() {
		this->state = true;
		this->setTexture(pressedTexture);
		this->action(this->state);
	}

	void reset() {
		this->state = false;
		this->setTexture(normalTexture);
	}

	void draw(std::shared_ptr<sf::RenderWindow> window) {
		window->draw(*this);
	}
};

class PopUpWindow : public sf::RenderWindow {
private:
	sf::Text text;
	std::vector<Button> buttons;
public:
	PopUpWindow(sf::Vector2u canvasSize, std::string title, std::string text, std::vector<Button> buttons) :
		RenderWindow(sf::VideoMode(canvasSize.x, canvasSize.y), title,
			sf::Style::Titlebar | sf::Style::Close)
	{
		int endOfPrevious;
		this->buttons = buttons;
		unsigned int spacing;
		unsigned int buttonWidth = this->buttons[0].getTexture()->getSize().x;

		this->text.setString(text);

		spacing =
			(canvasSize.x - buttonWidth) /
			(this->buttons.size() - 1) - buttonWidth;

		for (int i = 0; i < this->buttons.size(); i++) {
			if (i == 0) {
				this->buttons[0].setPosition(sf::Vector2f(canvasSize.x / 10, canvasSize.y / 2));
			}
			else if (i == this->buttons.size() - 1) {
				this->buttons[i].setPosition(sf::Vector2f(canvasSize.x - (canvasSize.x / 10), canvasSize.y / 2));
			}
			else {
				this->buttons[i].setPosition(sf::Vector2f(endOfPrevious + spacing, canvasSize.y / 2));
			}

			endOfPrevious = this->buttons[i].getPosition().x + buttonWidth;
		}
	}

	// Allow no exceptions from destructor to match sf::RenderWindow
	virtual ~PopUpWindow() throw();
};