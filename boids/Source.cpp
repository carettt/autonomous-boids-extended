#include "boid.h"
#include "flocks.h"
#include "channel.h"

// Struct to hold FPS statistics for event handler thread
struct Stats {
    double peakFPS;
    std::queue<double> lastFrames;
};

void displayResults(double peakFPS, std::queue<double> lastFrames) {
    // Function to display peak (all time) and average fps (over the last 10 frames)

    double averageFPS;
    int frameCount = lastFrames.size();
    
    for (; !lastFrames.empty(); lastFrames.pop()) {
        averageFPS += lastFrames.front() / frameCount;
    }

    std::cout << "Peak FPS: " << peakFPS << ", Average FPS (<" << frameCount << " frames): " << averageFPS << "\n";
}

// Main function
int main() {
    // Seed and initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Set canvas size
    const sf::Vector2u canvasSize(1920, 1080);

    // Initialize random distributions
    std::uniform_real_distribution<float> rand_x(0, canvasSize.x);
    std::uniform_real_distribution<float> rand_y(0, canvasSize.y);
    std::uniform_real_distribution<float> rand_v(-200, 200);

    const std::string title = "CMP202 boid simulation";

    // Intialize shared pointer to render window
    std::shared_ptr<sf::RenderWindow> window(new sf::RenderWindow());

    // Initialize delta timepoints and deltaTime
    std::chrono::steady_clock::time_point deltaStart;
    std::chrono::steady_clock::time_point deltaStop;

    double deltaTime = NULL;

    // Initialize FPS trackers
    std::queue<double> lastFrames;
    double peakFPS;

    // FPS Limit for debugging with < 15 boids
    // Without limit, deltaTime is too small and causes unexpected behaviour
    // -1 limit is unlimited FPS
    double maxFPS = -1;
    double sleepTime;

    // Initialize input variables
    char selectionInput;
    char deviceSelectionInput;
    bool valid = true;

    // Event handler thread and channel
    auto [tx, rx] = make_channel<Stats>();

    std::thread handler([window, &rx]() {
        Stats s = rx.read().value();

        displayResults(s.peakFPS, s.lastFrames);
        exit(0);
     });

    // Initialize sequential flock
    Flock sequential([&rand_x, &rand_y, &rand_v, &gen](int i) {
        return Boid(rand_x(gen), rand_y(gen),
        5.f, // radius
        200.f, // top speed
        sf::Vector2f(rand_v(gen), rand_v(gen)), // initial velocity
        15.f); // visibility
        }, 2.f, 0.25f, 0.25f, gen, window); // weights (separation, cohesion, alignment), gen, window ptr

    // Initialize CPU parallelized flock
    //CPUFlock cpu([&rand_x, &rand_y, &rand_v, &gen](int i) {
    //    return Boid(rand_x(gen), rand_y(gen),
    //    5.f, // radius
    //    200.f, // top speed
    //    sf::Vector2f(rand_v(gen), rand_v(gen)), // initial velocity
    //    15.f); // visibility
    //    }, 2.f, 0.25f, 0.25f, // weights (separation, cohesion, alignment)
    //    gen, window, 4); // window ptr, splits, deltaChannel consumer

    // Initialize naively CPU parallelized flock
    NaiveCPUFlock cpu([&rand_x, &rand_y, &rand_v, &gen](int i) {
        return Boid(rand_x(gen), rand_y(gen),
        5.f, // radius
        200.f, // top speed
        sf::Vector2f(rand_v(gen), rand_v(gen)), // initial velocity
        15.f); // visibility
        }, 2.f, 0.25f, 0.25f, gen, window, 32);

    // Initialize CPU parallelised flock
    GPUFlock gpu([&rand_x, &rand_y, &rand_v, &gen](int i) {
        return Boid(rand_x(gen), rand_y(gen),
        5.f, // radius
        200.f, // top speed
        sf::Vector2f(rand_v(gen), rand_v(gen)), // initial velocity
        15.f); // visibility
        }, 2.f, 0.25f, 0.25f, // weights (separation, cohesion, alignment)
        gen, window); // window ptr

    // Initialize runtime polymorphic flock
    Flock* flock = nullptr;

    // Uncomment following line to headcount boids split into chunks
    //std::cout << "flock size: " << cpu.size << "\nheadcount: " << cpu.countBoids() << std::endl;

    // Execution mode selection
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

    // Device selection if GPU execution mode
    if (selectionInput == '2') {
        do {
            sycl::gpu_selector g;
            sycl::cpu_selector c;

            std::cout << "[0] - GPU Device: " << g.select_device().get_info<sycl::info::device::name>() <<
                " Vendor: " << g.select_device().get_info<sycl::info::device::vendor>() <<
                " Max Compute Units: " << g.select_device().get_info<sycl::info::device::max_compute_units>() << "\n";
            std::cout << "[1] - CPU Device: " << c.select_device().get_info<sycl::info::device::name>() <<
                " Vendor: " << c.select_device().get_info<sycl::info::device::vendor>() <<
                " Max Compute Units: " << c.select_device().get_info<sycl::info::device::max_compute_units>() << "\n";
            std::cout << "Please select device to use: ";
            std::cin >> deviceSelectionInput;

            valid = true;

            switch (deviceSelectionInput) {
            case '0':
                gpu.setDevice(g.select_device());
                break;
            case '1':
                gpu.setDevice(c.select_device());
                break;
            default:
                std::cout << "Invalid selection ! Try again !\n";
                valid = false;
                break;
            }
        } while (!valid);
    }

    window->create(sf::VideoMode(canvasSize.x, canvasSize.y),
        title,
        sf::Style::Titlebar | sf::Style::Close);

    // Disable v-sync to unlimit framerate (may not work depending on graphics drivers, make sure to disable from video driver's control panel
    window->setVerticalSyncEnabled(false);
    window->setFramerateLimit(0);

    // Request focus to simulation window
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
                tx.write({ peakFPS, lastFrames });
                break;
            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::Key::Space) {
                    tx.write({ peakFPS, lastFrames });
                }
                break;
            }
        }

        // Clear window with background color
        window->clear(sf::Color(50, 50, 50, 255));

        //! [ --- GRAPHICS CODE FROM HERE --- ]

        // Update flock with delta time, update function handles first frame (NULL deltaTime)
        flock->update(deltaTime);

        //! [ --- STOP GRAPHICS CODE HERE --- ]

        // Render all queued objects
        window->display();

        //! [ --- STOP CODE HERE --- ]

        // End delta timer and set deltaTime
        deltaStop = std::chrono::high_resolution_clock::now();
        deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(deltaStop - deltaStart).count() / 1000.0;

        // Push new instant FPS to lastFrames and limit to 10 frames in queue
        lastFrames.push(1 / deltaTime);
        if (lastFrames.size() > 10) lastFrames.pop();

        // Record peak FPS
        if (deltaTime && (1 / deltaTime) > peakFPS) {
            peakFPS = (1 / deltaTime);
        }

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