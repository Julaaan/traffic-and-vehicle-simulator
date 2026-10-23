#include "vsim/DynamicsValidation.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv) {
    try {
        std::filesystem::path reference,report;
        for (int i=1; i<argc; ++i) {
            const std::string arg=argv[i];
            if (arg=="--help") { std::cout<<"sim_validate --reference FILE [--report FILE]\nReference CSV uses SI units and rear-axle pose; input commands are held until the next sample.\n"; return 0; }
            if (i+1==argc) throw std::invalid_argument("Missing value for " + arg);
            if (arg=="--reference") reference=argv[++i]; else if (arg=="--report") report=argv[++i]; else throw std::invalid_argument("Unknown option: " + arg);
        }
        if (reference.empty()) throw std::invalid_argument("--reference FILE is required");
        vsim::validateOutputPaths({report},{reference});
        std::ifstream manifest(reference);
        if (!manifest) throw std::runtime_error("Cannot open validation manifest: " + reference.string());
        const auto definition=nlohmann::json::parse(manifest);
        vsim::validateOutputPaths({report},{reference.parent_path()/definition.at("trace").get<std::string>()});
        const auto result=vsim::validateDynamicsReference(reference); vsim::writeDynamicsValidation(report,result);
        std::cout<<result.name<<" | "<<result.reference_kind<<" | "<<result.model<<" | "<<(result.passed ? "PASS" : "FAIL")<<'\n';
        for (const auto& m : result.metrics) std::cout<<m.name<<": RMSE "<<m.rmse<<", max "<<m.maximum<<' '<<m.unit<<" | "<<(m.passed ? "PASS" : "FAIL")<<'\n';
        return result.passed ? 0 : 2;
    } catch (const std::exception& error) { std::cerr<<"Error: "<<error.what()<<'\n'; return 1; }
}
