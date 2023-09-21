#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

/*
 * LOGS:
 *
 * - Load ROMS
 * - Read|Write bytes
 * @ Cpu opcodes
 */

// Will delete later
using namespace std;

// Shorthands
// *********************************
#define u8  uint8_t
#define u16  uint16_t
#define u32 uint32_t
// *********************************

// Flags
// *********************************
#define FLAG_ZERO (0x80)
#define FLAG_N    (0x40) // Subtraction flag
#define FLAG_H    (0x10) // Half carry flag
#define FLAG_C    (0x20) // Carry flag
// *********************************

// https://gbdev.io/pandocs/The_Cartridge_Header.html
namespace header {
    const int entry_point       = 0x0100; const int logo              = 0x0104;
    const int title             = 0x0134; const int manufacturer_code = 0x013F;
    const int cgb_flag          = 0x0143; const int new_license_code  = 0x0144;
    const int sgb_flag          = 0x0146; const int cartridge_type    = 0x0147;
    const int rom_size          = 0x0148; const int ram_size          = 0x0149;
    const int destination_code  = 0x014A; const int old_license_code  = 0x014B;
    const int version_number    = 0x014C; const int header_checksum   = 0x014D;
    const int global_checksum   = 0x014E;}

// For now, we are only doing no MBC
// This is the cartridge part
// *********************************
const char *fp = "roms/drmario.gb";
u8 version;
// *********************************

namespace Cartridge
{
    // https://gbdev.gg8.se/wiki/articles/Memory_Bank_Controllers
    // we don't need to worry about ROM banking right now
    u8 ROM[0x4000 * 2]; // 0x4000 - 0x7FFF
    u8 IO[0x80]; // FF00 - FF7F
    u8 HRAM[0xFFFF-0xFF80];
    u8 SRAM[0x2000];
    bool loaded = false;

    constexpr size_t TITLE_LENGTH = 11;
    char title[11];

    void LoadRomTitle() {
        for (auto i=0u; i<TITLE_LENGTH; ++i) {
            title[i] = ROM[header::title+i];
        }
    }

    void LoadRom(const char *filename) {
        FILE *f;
        f = fopen(filename, "rb");
        fread(ROM, 0x4000, 1, f);
        printf("Completed loading ROM: %s\n", filename);
        LoadRomTitle();
        fclose(f);

        loaded = true;
    }
}

namespace CPU
{
    struct Register {
        u16 SP, PC;
        // Registers, ordered Low first, then High
        union {
            u16 AF;
            struct {
                u8 F;
                u8 A;};};
        union {
            u16 BC;
            struct {
                u8 C;
                u8 B;};};
        union {
            u16 DE;
            struct {
                u8 E;
                u8 D;};};
        union {
            u16 HL;
            struct {
                u8 L;
                u8 H;};};
    } registers;

    // http://gameboy.mongenel.com/dmg/asmmemmap.html
    u8 RAM[0x1000]; // Internal RAM

    template<bool write> u8 MemAccess(u16 addr, u8 value=0);
    u8 RB(u16 addr) { return MemAccess<false>(addr); }
    u8 WB(u16 addr, u8 value) { return MemAccess<true>(addr, value); }
    void tick();
}

namespace IO
{
    // https://gbdev.io/pandocs/Joypad_Input.html#ff00--p1joyp-joypad
    struct {
        union {
            u8 keys;
            struct {
                unsigned int P10 : 1; // Right or A
                unsigned int P11 : 1; // Left  or B
                unsigned int P12 : 1; // Up    or Select
                unsigned int P13 : 1; // Down  or Start
                unsigned int P14 : 1; // Direction buttons
                unsigned int P15 : 1; // Action    buttons
            };};
    } joypad;
    u8 joy_current;
}

namespace PPU
{
    u8 VRAM[0x2000];
    u8 OAM[0xA0];

    struct {
        u8 control; // FF40
        u8 status;  // FF41
        u8 SCY;
        u8 SCX;
        u8 LY;      // FF44
        u8 LYC;
        u8 WY;      // Y Position
        u8 WX;      // X Position + 7
    } lcd;
}

namespace CPU
{
    void tick() {
        // Call PPU::tick()
    }

    struct {
        u8 IME;
        // 0xFFFF
        union {
            u8 IE;
            struct {
                unsigned int EVBLANK : 1;
                unsigned int ELCD    : 1;
                unsigned int ETimer  : 1;
                unsigned int ESerial : 1;
                unsigned int EJoypad : 1;
            };};
        // 0xFF0F
        union {
            u8 IF;
            struct {
                unsigned int FVBLANK : 1;
                unsigned int FLCD    : 1;
                unsigned int FTimer  : 1;
                unsigned int FSerial : 1;
                unsigned int FJoypad : 1;
            };};
    } interrupts;

    struct {
        u8 DIV;     // FF04
        u8 TIMA;    // FF05
        u8 TMA;     // FF06
        union {
            u8 TAC; // FF07
            struct {
                unsigned int Speed  : 2;
                unsigned int Enable : 1;
            };};
    } timer;

    template<bool write> u8 MemAccess(u16 addr, u8 v) {
        tick();
        // macros??

        #define writeOrRet(what) u8 &r = what; if (!write) return r; r=v
        if (addr <= 0x3FFF) {
            writeOrRet(Cartridge::ROM[addr]);
            printf("WARNING: Just wrote %d @%X\n", v, addr);}
        else if (addr <= 0x7FFF) {
            writeOrRet(Cartridge::ROM[addr]);
            printf("WARNING: Just wrote %d @%X\n", v, addr);}
        else if (addr <= 0x9FFF) {
            writeOrRet(PPU::VRAM[addr - 0x8000]);}
        else if (addr <= 0xBFFF) {
            printf("WARNING: The memory here is not supposed to be accessed by MBC0 ROMS\n");}
        else if (addr <= 0xDFFF) {
            fprintf(stderr, "ERROR: WRAM is unimplemented\n");
            exit(1);
        }
        else if (0xFE00 <= addr && addr <= 0xFE9F) {
            writeOrRet(PPU::OAM[addr - 0XFE00]);}
        else if (addr <= 0xFEFF) {
            // Prohibited sector
            return 0;}
        else if (addr <= 0xFF7F) {
            // https://gbdev.io/pandocs/Memory_Map.html#io-ranges
            switch (addr) {
            case 0xFF00: {
                if (!write) {
                    // TODO: Read more source codes for the joypad
                    // p14: Direction buttons
                    if (!(IO::joypad.keys & 0x10)) {
                        return ~(IO::joy_current & 0xF);}
                    if (!(IO::joypad.keys & 0x20)) {
                        return ~((IO::joy_current & 0xF0) >> 4);}
                } else
                    IO::joypad.keys = v;
            } break;
            case 0xFF04: {
                if (!write) return timer.DIV; else {
                    timer.DIV = 0;
                    timer.TIMA = timer.TMA;
                }
            } break;
            case 0xFF05: {writeOrRet(timer.TIMA);} break;
            case 0xFF06: {writeOrRet(timer.TMA);} break;
            case 0xFF07: {
                if (!write) return timer.TAC; else {
                    int old_freq = timer.TAC & 0x3;
                    timer.TAC = v;
                    printf("TAC: %02X\n", timer.TAC);
                    int new_freq = timer.TAC & 0x3;
                    if (old_freq != new_freq) {
                        fprintf(stderr, "ERROR: Should update timer counter here\n");
                        exit(1);
                    }
                }
            } break;
            case 0xFF0F: {writeOrRet(timer.TMA);} break;
            case 0xFF40: {writeOrRet(PPU::lcd.control);} break;
            case 0xFF41: {writeOrRet(PPU::lcd.status);} break;
            case 0xFF42: {writeOrRet(PPU::lcd.SCY);} break;
            case 0xFF43: {writeOrRet(PPU::lcd.SCX);} break;
            case 0xFF44: {writeOrRet(PPU::lcd.LY);} break;
            case 0xFF45: {writeOrRet(PPU::lcd.LYC);} break;
            case 0xFF4A: {writeOrRet(PPU::lcd.WY);} break;
            case 0xFF4B: {writeOrRet(PPU::lcd.WX);} break;
            default: {writeOrRet(Cartridge::IO[addr - 0xFF00]);}}}
        else if (addr <= 0xFFFF) {
            if (addr == 0xFFFF) {
                writeOrRet(interrupts.IE);
            } else {
                fprintf(stderr, "ERROR: HRAM is unimplemented\n");
                exit(1);}}
        else {
            fprintf(stderr, "ERROR: unreachable [read|write] byte area\n");
            exit(1);}
        return 0;
    }

    // Bisqwit's Nesemu:        https://bisqwit.iki.fi/jutut/kuvat/programming_examples/nesemu1/nesemu1.cc
    // Explanation for Bisqwit: https://www.maizure.org/projects/decoded-bisqwit-nes-emulator/nesemu1_decoded.txt
    // BASE64 Encoder:          https://cryptii.com/pipes/base64-to-hex
    template<u8 op>
    void Ins() {
        return;
    }

    // this would be the ``loop''
    void Op() {
        u8 op = RB(registers.PC++);

        // Today I learned about the paste operator a.k.a. ## in c++
        #define c(n) Ins<0x10>, Ins<0x10>,
        #define o(n) c(n)c(n+2)c(n+4)c(n+6)
        static void (*i[0x108])() = {
            o(00)o(08)o(10)
        };
        #undef o
        #undef c
    }
}

int main()
{
    // Initial state
    // resetting everything
    memset(Cartridge::ROM,  0, sizeof(Cartridge::ROM));
    memset(PPU::VRAM,       0, sizeof(PPU::VRAM));
    memset(Cartridge::SRAM, 0, sizeof(Cartridge::SRAM));
    memset(PPU::OAM,        0, sizeof(PPU::OAM));
    memset(Cartridge::HRAM, 0, sizeof(Cartridge::HRAM));

    // Registers
    CPU::registers.A  = 0x01;   CPU::registers.F  = 0xB0;
    CPU::registers.B  = 0x00;   CPU::registers.C  = 0x13;
    CPU::registers.D  = 0x00;   CPU::registers.E  = 0xD8;
    CPU::registers.H  = 0x01;   CPU::registers.L  = 0x4D;
    CPU::registers.SP = 0xFFFE; CPU::registers.PC = 0x0100;

    // Interrupts
    IO::joypad.keys    = 0xFF;  CPU::interrupts.IME = 0;
    CPU::interrupts.IE = 0;     CPU::interrupts.IF  = 0x01;

    // Timing
    CPU::timer.DIV = 0xAB;      CPU::timer.TIMA = 0;
    CPU::timer.TMA = 0;         CPU::timer.TAC  = 0xF8;

    // LCD
    PPU::lcd.LY     = 0;        PPU::lcd.control = 0x91;
    PPU::lcd.status = 0x85;     PPU::lcd.SCY     = 0;
    PPU::lcd.SCX    = 0;        PPU::lcd.LYC     = 0;
    PPU::lcd.WY     = 0;        PPU::lcd.WX      = 0;

    // Palettes
    CPU::WB(0xFF47, 0xFC);
    CPU::WB(0xFF48, 0xFF);
    CPU::WB(0xFF49, 0xFF);
    // end of reset

    printf("-------------u(r mom's)-Boy-------------\n");

    Cartridge::LoadRom(fp);
    u8 version_code = Cartridge::ROM[header::version_number];
    version = version_code;

    printf("Title:\t\t %s (version %d)\n", Cartridge::title, version);

    // Cartridge and MBC (which is no MBC currently)

    return(0);
}
