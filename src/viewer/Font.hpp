#pragma once
#include <SDL.h>
#include <array>
#include <memory>
#include <string>

namespace vsim::viewer {
enum class TextSize { Small, Large };
class Fonts {
public:
    explicit Fonts(SDL_Renderer* renderer);
    ~Fonts();
    Fonts(const Fonts&) = delete;
    Fonts& operator=(const Fonts&) = delete;
    void draw(SDL_Renderer* renderer, int x, int y, const std::string& text, SDL_Color color,
              TextSize size = TextSize::Small) const;
    int width(const std::string& text, TextSize size = TextSize::Small) const;
    int capHeight(TextSize size = TextSize::Small) const;
private:
    struct Face;
    const Face& face(TextSize size) const;
    std::array<std::unique_ptr<Face>, 2> faces_;
};
}
