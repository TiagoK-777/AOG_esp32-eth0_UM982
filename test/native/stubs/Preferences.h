#pragma once
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>
inline std::map<std::string, std::vector<uint8_t>> fakeNvs;
inline bool fakeNvsFail = false;
class Preferences {
public:
    bool begin(const char* name, bool) { return std::string(name) == "aog"; }
    void end() {}
    bool isKey(const char* key) { return fakeNvs.count(key); }
    template<class T> size_t put(const char* key, T value) {
        if (fakeNvsFail) return 0;
        auto& bytes = fakeNvs[key]; bytes.resize(sizeof(T));
        std::memcpy(bytes.data(), &value, sizeof(T)); return sizeof(T);
    }
    template<class T> T get(const char* key, T fallback) {
        auto found = fakeNvs.find(key);
        if (found == fakeNvs.end() || found->second.size() != sizeof(T)) return fallback;
        T value; std::memcpy(&value, found->second.data(), sizeof(T)); return value;
    }
    size_t putUChar(const char* k, uint8_t v) { return put(k,v); }
    size_t putShort(const char* k, int16_t v) { return put(k,v); }
    size_t putFloat(const char* k, float v) { return put(k,v); }
    size_t putInt(const char* k, int32_t v) { return put(k,v); }
    uint8_t getUChar(const char* k, uint8_t v) { return get(k,v); }
    int16_t getShort(const char* k, int16_t v) { return get(k,v); }
    float getFloat(const char* k, float v) { return get(k,v); }
};
