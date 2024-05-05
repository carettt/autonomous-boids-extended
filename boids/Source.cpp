#include "boid.h"
#include "flocks.h"

// Sequential

enum selector { SEQ, CPU, GPU };

// Main function
int main() {
    // Seed and initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Set canvas size
    const sf::Vector2u canvasSize(1920, 1080);

    std::uniform_real_distribution<float> rand_x(0, canvasSize.x);
    std::uniform_real_distribution<float> rand_y(0, canvasSize.y);
    std::uniform_real_distribution<float> rand_v(-200, 200);

    const std::string title = "CMP202 boid simulation";

    // Intiialize shared pointer to render window
    std::shared_ptr<sf::RenderWindow> window(new sf::RenderWindow());

    // Initialize delta timepoints and deltaTime
    std::chrono::steady_clock::time_point deltaStart;
    std::chrono::steady_clock::time_point deltaStop;

    float deltaTime = NULL;

    // FPS Limit for debugging with < 15 boids
    // Without limit, deltaTime is too small and causes unexpected behaviour
    // -1 limit is unlimited FPS
    float maxFPS = -1;
    float sleepTime;

    char selectionInput;
    bool valid = true;

    // Initialize sequential flock
    Flock sequential([&rand_x, &rand_y, &rand_v, &gen](int i) {
        return Boid(rand_x(gen), rand_y(gen),
        5.f, // radius
        200.f, // top speed
        sf::Vector2f(rand_v(gen), rand_v(gen)), // initial velocity
        15.f); // visibility
        }, 2.f, 0.25f, 0.25f, gen, window); // weights (separation, cohesion, alignment), gen, window ptr

    // Initialize CPU parallelised flock
    CPUFlock cpu([&rand_x, &rand_y, &rand_v, &gen](int i) {
        return Boid(rand_x(gen), rand_y(gen),
        5.f, // radius
        200.f, // top speed
        sf::Vector2f(rand_v(gen), rand_v(gen)), // initial velocity
        15.f); // visibility
        }, 2.f, 0.25f, 0.25f, // weights (separation, cohesion, alignment)
        gen, window, 4); // window ptr, splits, deltaChannel consumer

    // Initialize CPU parallelised flock
    GPUFlock gpu([&rand_x, &rand_y, &rand_v, &gen](int i) {
        return Boid(rand_x(gen), rand_y(gen),
        5.f, // radius
        200.f, // top speed
        sf::Vector2f(rand_v(gen), rand_v(gen)), // initial velocity
        15.f); // visibility
        }, 2.f, 0.25f, 0.25f, // weights (separation, cohesion, alignment)
        gen, window, 4); // window ptr, splits, deltaChannel consumer

    Flock* flock = nullptr;

    cpu.localizeBoids();

    // Uncomment following line to headcount boids split into chunks
    std::cout << "flock size: " << cpu.size << "\nheadcount: " << cpu.countBoids() << std::endl;

    do {
        std::cout << "Please select execution mode (SEQ, CPU, GPU) [0/1/2]: ";
        std::cin >> selectionInput;
        std::cout << std::endl;

        valid = true;

        switch (selectionInput) {
        case '0':
            flock = &sequential;
            break;
        case '1':
            flock = &cpu;
            break;
        case '2':
            flock = &gpu;
            break;
        default:
            std::cout << "Invalid selection ! Try again !" << std::endl;
            valid = false;
            break;
        }
    } while (!valid);

    window->create(sf::VideoMode(canvasSize.x, canvasSize.y),
        title,
        sf::Style::Titlebar | sf::Style::Close);
    window->requestFocus();

    // Window loop
    while (window->isOpen())
    {
        // Start delta timer
        deltaStart = std::chrono::high_resolution_clock::now();
        //! [ --- CODE FROM HERE --- ]

        // Check for events
        sf::Event event;
        while (window->pollEvent(event))
        {
            // Event handlers
            switch (event.type) {
            case sf::Event::Closed:
                window->close();
                exit(0);
                break;
            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::Key::Space) {
                    window->close();
                    exit(0);
                }
                break;
            }
        }

        // Clear window with background color
        window->clear(sf::Color(50, 50, 50, 255));

        //! [ --- GRAPHICS CODE FROM HERE --- ]

        // Only update flock after first frame, when deltaTime has a value
        flock->update(deltaTime);

        //! [ --- STOP GRAPHICS CODE HERE --- ]

        // Render all queued objects
        window->display();

        //! [ --- STOP CODE HERE --- ]

        // End delta timer and set deltaTime
        deltaStop = std::chrono::high_resolution_clock::now();
        deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(deltaStop - deltaStart).count() / 1000.f;

        // Limit fps by sleeping for difference between desired frame time and delta time
        sleepTime = (1 / maxFPS) - deltaTime;

        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::duration<float>(sleepTime));
            deltaTime += sleepTime;
        }

        // Set title to title + FPS
        window->setTitle(title + ", " + std::to_string((int)(1 / deltaTime)) + "FPS");
    }

    return 0;
}