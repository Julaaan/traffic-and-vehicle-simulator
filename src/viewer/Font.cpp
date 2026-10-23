#include "Font.hpp"
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>
#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include <stb_truetype.h>

namespace vsim::viewer {
namespace {
constexpr int first_glyph = 32, glyph_count = 224, atlas_size = 512;

std::vector<std::filesystem::path> candidates(bool semibold) {
    std::vector<std::filesystem::path> paths;
    if (const char* windows = SDL_getenv("WINDIR")) {
        const auto fonts = std::filesystem::path(windows) / "Fonts";
        if (semibold) paths.push_back(fonts / "seguisb.ttf");
        paths.push_back(fonts / "segoeui.ttf");
    }
    for (const char* directory : {"/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/TTF", "/usr/share/fonts/dejavu"}) {
        if (semibold) paths.push_back(std::filesystem::path(directory) / "DejaVuSans-Bold.ttf");
        paths.push_back(std::filesystem::path(directory) / "DejaVuSans.ttf");
    }
    if (semibold) paths.push_back("/System/Library/Fonts/Supplemental/Arial Bold.ttf");
    paths.push_back("/System/Library/Fonts/Supplemental/Arial.ttf");
    return paths;
}
std::vector<unsigned char> readFont(bool semibold) {
    for (const auto& path : candidates(semibold)) {
        std::ifstream input(path, std::ios::binary);
        if (input) return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    }
    throw std::runtime_error("No usable system font found (Segoe UI, DejaVu Sans or Arial)");
}
int nextGlyph(const std::string& text, std::size_t& i) {
    const auto byte = [&](std::size_t k) { return k < text.size() ? static_cast<unsigned>(static_cast<unsigned char>(text[k])) : 0u; };
    const unsigned lead = byte(i);
    unsigned code = '?';
    if (lead < 0x80) { code = lead; i += 1; }
    else if ((lead & 0xE0) == 0xC0) { code = ((lead & 0x1F) << 6) | (byte(i + 1) & 0x3F); i += 2; }
    else if ((lead & 0xF0) == 0xE0) { code = ((lead & 0x0F) << 12) | ((byte(i + 1) & 0x3F) << 6) | (byte(i + 2) & 0x3F); i += 3; }
    else i += 1;
    return code >= first_glyph && code < first_glyph + glyph_count ? static_cast<int>(code) : '?';
}
}

struct Fonts::Face {
    std::vector<unsigned char> data;
    stbtt_fontinfo info{};
    std::array<stbtt_packedchar, glyph_count> glyphs{};
    float scale{};
    int cap{};
    SDL_Texture* atlas{};
    Face(SDL_Renderer* renderer, std::vector<unsigned char> font, float em) : data(std::move(font)) {
        if (!stbtt_InitFont(&info, data.data(), stbtt_GetFontOffsetForIndex(data.data(), 0)))
            throw std::runtime_error("Unreadable font file");
        scale = stbtt_ScaleForMappingEmToPixels(&info, em);
        int x0{}, y0{}, x1{}, y1{};
        stbtt_GetCodepointBox(&info, 'H', &x0, &y0, &x1, &y1);
        cap = static_cast<int>(std::lround(y1 * scale));
        std::vector<unsigned char> coverage(atlas_size * atlas_size);
        stbtt_pack_context pack{};
        if (!stbtt_PackBegin(&pack, coverage.data(), atlas_size, atlas_size, 0, 1, nullptr))
            throw std::runtime_error("Font atlas allocation failed");
        stbtt_PackSetOversampling(&pack, 2, 1);
        const int packed = stbtt_PackFontRange(&pack, data.data(), 0, STBTT_POINT_SIZE(em), first_glyph, glyph_count, glyphs.data());
        stbtt_PackEnd(&pack);
        if (!packed) throw std::runtime_error("Font atlas is too small");
        std::vector<std::uint32_t> pixels(coverage.size());
        for (std::size_t i = 0; i < coverage.size(); ++i) pixels[i] = (std::uint32_t{coverage[i]} << 24) | 0x00FFFFFFu;
        atlas = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, atlas_size, atlas_size);
        if (!atlas) throw std::runtime_error(SDL_GetError());
        SDL_UpdateTexture(atlas, nullptr, pixels.data(), atlas_size * 4);
        SDL_SetTextureBlendMode(atlas, SDL_BLENDMODE_BLEND);
        SDL_SetTextureScaleMode(atlas, SDL_ScaleModeLinear);
    }
    ~Face() { SDL_DestroyTexture(atlas); }
    Face(const Face&) = delete;
    Face& operator=(const Face&) = delete;
    float kerning(int previous, int code) const {
        return previous ? scale * static_cast<float>(stbtt_GetCodepointKernAdvance(&info, previous, code)) : 0.0f;
    }
};

Fonts::Fonts(SDL_Renderer* renderer)
    : faces_{std::make_unique<Face>(renderer, readFont(false), 13.0f), std::make_unique<Face>(renderer, readFont(true), 20.0f)} {}
Fonts::~Fonts() = default;
const Fonts::Face& Fonts::face(TextSize size) const { return *faces_[size == TextSize::Large ? 1 : 0]; }
int Fonts::capHeight(TextSize size) const { return face(size).cap; }
int Fonts::width(const std::string& text, TextSize size) const {
    const auto& f = face(size);
    float x = 0.0f;
    for (std::size_t i = 0, previous = 0; i < text.size();) {
        const int code = nextGlyph(text, i);
        x += f.kerning(static_cast<int>(previous), code) + f.glyphs[static_cast<std::size_t>(code - first_glyph)].xadvance;
        previous = static_cast<std::size_t>(code);
    }
    return static_cast<int>(std::ceil(x));
}
void Fonts::draw(SDL_Renderer* renderer, int x, int y, const std::string& text, SDL_Color color, TextSize size) const {
    const auto& f = face(size);
    SDL_SetTextureColorMod(f.atlas, color.r, color.g, color.b);
    SDL_SetTextureAlphaMod(f.atlas, color.a);
    float pen_x = static_cast<float>(x), pen_y = static_cast<float>(y + f.cap);
    for (std::size_t i = 0, previous = 0; i < text.size();) {
        const int code = nextGlyph(text, i);
        pen_x += f.kerning(static_cast<int>(previous), code);
        stbtt_aligned_quad quad{};
        stbtt_GetPackedQuad(f.glyphs.data(), atlas_size, atlas_size, code - first_glyph, &pen_x, &pen_y, &quad, 0);
        const auto texel = [](float coordinate) { return static_cast<int>(std::lround(coordinate * atlas_size)); };
        const SDL_Rect source{texel(quad.s0), texel(quad.t0), texel(quad.s1) - texel(quad.s0), texel(quad.t1) - texel(quad.t0)};
        const SDL_FRect target{quad.x0, quad.y0, quad.x1 - quad.x0, quad.y1 - quad.y0};
        if (source.w > 0 && source.h > 0) SDL_RenderCopyF(renderer, f.atlas, &source, &target);
        previous = static_cast<std::size_t>(code);
    }
}
}
