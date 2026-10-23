#include "vsim/OsmImporter.hpp"
#include <tinyxml2.h>
#include <array>
#include <fstream>
#include <stdexcept>

namespace vsim {
namespace {
OsmId idAttribute(const tinyxml2::XMLElement& element, const char* name) {
    OsmId value{};
    if (element.QueryInt64Attribute(name, &value) != tinyxml2::XML_SUCCESS)
        throw std::runtime_error(std::string("Invalid or missing ") + name + " at XML line " + std::to_string(element.GetLineNum()));
    return value;
}
OsmTags tags(const tinyxml2::XMLElement& element) {
    OsmTags result;
    for (auto tag = element.FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
        const char* key = tag->Attribute("k"); const char* value = tag->Attribute("v");
        if (!key || !value) throw std::runtime_error("OSM tag requires k and v attributes");
        result[key] = value;
    }
    return result;
}
bool deleted(const tinyxml2::XMLElement& element) {
    return element.Attribute("action", "delete") || element.Attribute("visible", "false");
}
std::array<double, 3> ecef(double latitude, double longitude) {
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || std::abs(latitude) > 90.0 || std::abs(longitude) > 180.0)
        throw std::invalid_argument("Invalid WGS84 coordinates");
    constexpr double a = 6378137.0;
    constexpr double e2 = 6.6943799901413165e-3;
    const double lat = latitude * pi / 180.0, lon = longitude * pi / 180.0;
    const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    return {n * std::cos(lat) * std::cos(lon), n * std::cos(lat) * std::sin(lon), n * (1.0 - e2) * std::sin(lat)};
}
}
MapProjection::MapProjection(double latitude, double longitude)
    : latitude_(latitude * pi / 180.0), longitude_(longitude * pi / 180.0) {
    const auto origin = ecef(latitude, longitude);
    origin_x_ = origin[0]; origin_y_ = origin[1]; origin_z_ = origin[2];
}
Vec2 MapProjection::project(double latitude, double longitude) const {
    const auto position = ecef(latitude, longitude);
    const double x = position[0] - origin_x_, y = position[1] - origin_y_, z = position[2] - origin_z_;
    return {-std::sin(longitude_) * x + std::cos(longitude_) * y,
        -std::sin(latitude_) * std::cos(longitude_) * x - std::sin(latitude_) * std::sin(longitude_) * y + std::cos(latitude_) * z};
}
OsmData readOsm(const std::filesystem::path& path) {
    try {
        std::ifstream input(path, std::ios::binary | std::ios::ate);
        if (!input) throw std::runtime_error("Cannot open file");
        const auto size = input.tellg();
        if (size < 0 || size > 256LL * 1024 * 1024) throw std::runtime_error("OSM XML must be at most 256 MiB; select a smaller map extract");
        std::string xml(static_cast<std::size_t>(size), '\0');
        input.seekg(0); input.read(xml.data(), static_cast<std::streamsize>(xml.size()));
        if (!input) throw std::runtime_error("Cannot read file");
        tinyxml2::XMLDocument document;
        if (document.Parse(xml.data(), xml.size()) != tinyxml2::XML_SUCCESS)
            throw std::runtime_error(std::string("Invalid OSM XML: ") + document.ErrorStr());
        const auto root = document.RootElement();
        if (!root || std::string(root->Name()) != "osm") throw std::runtime_error("Expected an <osm> root; only OSM XML snapshots are supported");
        OsmData data;
        for (auto element = root->FirstChildElement(); element; element = element->NextSiblingElement()) {
            if (deleted(*element)) continue;
            const std::string kind = element->Name();
            if (kind == "node") {
                OsmNode node; node.id = idAttribute(*element, "id");
                if (element->QueryDoubleAttribute("lat", &node.latitude) != tinyxml2::XML_SUCCESS
                    || element->QueryDoubleAttribute("lon", &node.longitude) != tinyxml2::XML_SUCCESS)
                    throw std::runtime_error("Node " + std::to_string(node.id) + " requires lat and lon");
                ecef(node.latitude, node.longitude);
                node.tags = tags(*element);
                if (!data.nodes.emplace(node.id, std::move(node)).second) throw std::runtime_error("Duplicate node id in OSM snapshot");
            } else if (kind == "way") {
                OsmWay way; way.id = idAttribute(*element, "id"); way.tags = tags(*element);
                for (auto nd = element->FirstChildElement("nd"); nd; nd = nd->NextSiblingElement("nd"))
                    way.nodes.push_back(idAttribute(*nd, "ref"));
                if (!data.ways.emplace(way.id, std::move(way)).second) throw std::runtime_error("Duplicate way id in OSM snapshot");
            } else if (kind == "relation") {
                OsmRelation relation; relation.id = idAttribute(*element, "id"); relation.tags = tags(*element);
                for (auto member = element->FirstChildElement("member"); member; member = member->NextSiblingElement("member")) {
                    const auto type = member->Attribute("type"), role = member->Attribute("role");
                    if (!type || !role) throw std::runtime_error("OSM relation member requires type and role");
                    relation.members.push_back({type, role, idAttribute(*member, "ref")});
                }
                if (!data.relations.emplace(relation.id, std::move(relation)).second) throw std::runtime_error("Duplicate relation id in OSM snapshot");
            }
        }
        return data;
    } catch (const std::exception& error) {
        throw std::runtime_error("OSM '" + path.string() + "': " + error.what());
    }
}
}
