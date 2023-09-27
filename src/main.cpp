#include <any>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <tuple>
#include <string>
#include <vector>

/*
 * LOGS:
 *
 * - Load ROMS
 * - Read|Write bytes
 * @ Cpu opcodes -> [current: 8 bit loading]
 * ^ Interrupts, and fix the infinite loop
 * - Accurate Timing
 * - PPU
 * - APU
 */

// Will delete later (when, I finish the PPU)
using namespace std;
using namespace std::placeholders;

// Shorthands
// *********************************
#define u8  uint8_t
#define u16  uint16_t
#define u32 uint32_t
// *********************************

// Flags
// *********************************
#define FLAG_Z    (1 << 7) // Zero flag
#define FLAG_N    (1 << 6) // Subtraction flag
#define FLAG_H    (1 << 5) // Half carry flag
#define FLAG_C    (1 << 4) // Carry flag
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
        fread(ROM, 0x4000 * 2, 1, f);
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

    template<bool write> unsigned MemAccess(u16 addr, u8 value=0);
    unsigned RB(u16 addr) { return MemAccess<false>(addr); }
    unsigned WB(u16 addr, u8 value) { return MemAccess<true>(addr, value); }
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

    unsigned ft(unsigned n) {
        unsigned r=0;
        for (int i=0; i<8*n; i+=8)
            r = (RB(registers.PC++)<<i) | r;
        return r;
    }

    void set_flag(int flag, bool t) {
        if (t) registers.F |= flag;
        else registers.F &= ~(flag);
    }

    // http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf
    void nop() {}

    void stop() {/* if (!interrupts.FJoypad) registers.PC--; */}
    // put value nn into addr
    void writeu16(u16 addr, u16 nn) {
        WB(addr, (u8)(nn & 0xFF));
        WB(addr, (u8)(nn >> 8));
    }

    void load16bit(u16 *addr, u16 nn) {*addr = nn;}

    template<typename T> // T is usually u8, with the exception of HL being u16 (see table r)
    void load8bit(T *addr, u8 n) {*addr = n;}

    // jump -127 -> +129 steps from the current address
    void jr(u8 d) {
        registers.PC += (int8_t)d;
    }

    void jp(u16 nn) {
        registers.PC = nn;
    }

    void add16(u16 target, u16 value) {
        int r = target + value;
        set_flag(FLAG_N, false);
        // set if carry on bit 11
        set_flag(FLAG_H, ((target&0xFFF) + (value&0xFFF) & 0x1000) != 0);
        // set if carry on bit 15
        set_flag(FLAG_C, r > 0xFFFF);
    }

    void addhl(u16 *val) {
        u16 r = registers.HL + *val;
        add16(registers.HL, *val);
        registers.HL = r;
    }

    // alu functions
    void aluADDA(u8 value) {
        u16 r       = (u16)registers.A + (u16)value;
        registers.A = (r & 0xFF);
        // set if result == 0                  // reset
        set_flag(FLAG_Z, (r & 0xFF)==0);       set_flag(FLAG_N, false);
        // set if carry from bit 7             // set if carry from bit 3
        set_flag(FLAG_C, r > 0xFF);            set_flag(FLAG_H, ((registers.A & 0xF) +
                                                                 (value & 0xF)) > 0xF);
    }

    void aluADCA(u8 value) {
        u16 carry   = !!(registers.F & FLAG_C);
        u16 r       = (u16)registers.A + (u16)value + carry;
        registers.A = (r & 0xFF);
        // set if result == 0                  // reset
        set_flag(FLAG_Z, (r & 0xFF)==0);       set_flag(FLAG_N, false);
        // set if carry from bit 7             // set if carry from bit 3
        set_flag(FLAG_C, r > 0xFF);            set_flag(FLAG_H, ((registers.A & 0xF) +
                                                                 (value + 0xF) +
                                                                 (u8)carry) > 0xF);
    }

    void aluSUB(u8 value) {
        int16_t s_A = (int16_t)registers.A,
                s_v = (int16_t)value;
        int16_t r   = s_A - s_v;
        registers.A = (u8)(r & 0xFF);
        // set if result == 0                  // set to true
        set_flag(FLAG_Z, (r & 0xFF)==0);       set_flag(FLAG_N, true);
        // set if no borrow                    // set if no borrow from bit 4
        set_flag(FLAG_C, r<0);                 set_flag(FLAG_H, ((s_A & 0xF) -
                                                                 (s_v & 0xF)) < 0);
    }

    void aluSBCA(u8 value) {
        int16_t s_A   = (int16_t)registers.A,
                s_v   = (int16_t)value,
                carry = !!(registers.F & FLAG_C);
        int16_t r     = s_A - s_v - carry;
        registers.A   = (u8)(r & 0xFF);
        // set if result == 0                  // set to true
        set_flag(FLAG_Z, (r & 0xFF)==0);       set_flag(FLAG_N, true);
        // set if no borrow                    // set if no borrow from bit 4
        set_flag(FLAG_C, r<0);                 set_flag(FLAG_H, ((s_A & 0xF) -
                                                                 (s_v & 0xF) - carry) < 0);
    }

    void aluAND(u8 value) {
        registers.A &= value;
        set_flag(FLAG_Z, registers.A==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, true);
        set_flag(FLAG_C, false);
    }

    void aluXOR(u8 value) {
        registers.A ^= value;
        set_flag(FLAG_Z, registers.A==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, false);
        set_flag(FLAG_C, false);
    }

    void aluOR(u8 value) {
        registers.A |= value;
        set_flag(FLAG_Z, registers.A==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, false);
        set_flag(FLAG_C, false);
    }

    // Compare A with n; This is basically a A - n subtraction but the results are thrown away
    void aluCP(u8 value) {
        int16_t r = registers.A - value;
        set_flag(FLAG_Z, r==0);
        set_flag(FLAG_N, true);
        set_flag(FLAG_H, ((registers.A & 0xF) - (value & 0xF)) < 0);
        set_flag(FLAG_C, r<0);
    }

    void rl(u8 *target, bool carry) {
        bool bit7 = (*target & 0x80) != 0;
        *target <<= 1;
        *target |= carry ? registers.F&FLAG_C : bit7;
        set_flag(FLAG_Z, *target==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, false);
        set_flag(FLAG_C, bit7);
    }

    void rr(u8 *target, bool carry) {
        bool bit1 = (*target & 0x1) != 0;
        *target >>= 1;
        *target |= carry ? (registers.F&FLAG_C)<<7 : bit1<<7;
        set_flag(FLAG_Z, *target==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, false);
        set_flag(FLAG_C, bit1!=0);
    }

    // Decimal adjust register A.
    // This instruction adjusts register A so that the
    // correct representation of Binary Coded Decimal (BCD)
    // is obtained.
    //
    // very confusing, I had to lookup different implementations
    // for this:
    // - https://github.com/ChanellR/Fatboy/blob/master/src/cpu.c#L396
    // - https://github.com/mattbruv/Gameboy-Emulator/blob/master/src/cpu.cpp#L750
    void daa() {
        int8_t additive = 0; u8 done = 0;

        if (!(registers.F & FLAG_N)) {
            // after an addition, adjust if (half-)carry occurred or if result
            // is out of bounds
            if ((registers.A & 0xF0)>0x80 && (registers.A & 0xF0)>0x9) {
                additive+=0x66;
                done     = 1;
                set_flag(FLAG_C, true);
            }
            if ((registers.F & FLAG_C || (registers.A & 0xF0)>0x90) && !done) {
                additive+=0x60;
                set_flag(FLAG_C, true);
            }
            if ((registers.F & FLAG_H || (registers.A & 0x0F)>0x09) && !done)
                additive+=0x06;
        } else {
            // after a subtraction, only adjust if (half-)carry occurred
            if (registers.F & FLAG_C)
                additive-=0x60;
            if (registers.F & FLAG_H)
                additive-=0x06;
            if (additive<=-0x60)
                set_flag(FLAG_C, true);
        }
        registers.A+=additive;

        set_flag(FLAG_Z, registers.A==0);
        set_flag(FLAG_H, false); // half-carry will always be cleared
    }

    void ret() {
        u16 lo = (u16)RB(registers.SP++);
        u16 hi = (u16)RB(registers.SP++);
        registers.PC = (hi << 0xF) | lo;
    }

    void cbRLC(u8 *target) {rl(target, false);}

    void cbRRC(u8 *target) {rr(target, false);}

    void cbRL(u8 *target) {rl(target, true);}

    void cbRR(u8 *target) {rr(target, true);}

    void cbSLA(u8 *target) {
        u8 r = *target << 1;
        set_flag(FLAG_Z, r==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, false);
        set_flag(FLAG_C, (*target & 0x80) != 0);
        *target = r;
    }

    void cbSR(u8 *target, bool includeTopBit) {
        bool isTopBitSet = *target & 0x80;
        u8 r;
        if (includeTopBit) {
            r = isTopBitSet ? *target>>1 | 0x80 : *target>>1;
        } else {
            r = *target>>1;
        }
        set_flag(FLAG_Z, r==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, false);
        set_flag(FLAG_C, (*target & 0x01) != 0);
        *target = r;
    }

    void cbSRA(u8 *target) {
        cbSR(target, true);
    }

    void cbSRL(u8 *target) {
        cbSR(target, false);
    }

    void cbSWAP(u8 *target) {
        u8 first = *target>>4,
        second = *target<<4;

        u8 swapped = first | second;
        *target = swapped;

        set_flag(FLAG_Z, *target==0);
        set_flag(FLAG_N, false);
        set_flag(FLAG_H, false);
        set_flag(FLAG_C, false);
    }

    void cbBIT(u8 target, u8 index) {
        set_flag(FLAG_Z, ((1<<index) & ~target) != 0);
        set_flag(FLAG_H, true);
        set_flag(FLAG_N, false);
    }

    void cbSET(u8 *target, u8 index) {
        *target = (*target | (1<<index));
    }

    void cbRES(u8 *target, u8 index) {
        *target = (~(1<<index) & *target);
    }

    void call(u16 value) {
        WB(--(registers.SP), (u8)(registers.PC >> 8));
        WB(--(registers.SP), registers.PC & 0xFF);
        registers.PC = value;
    }

    template<bool write> unsigned MemAccess(u16 addr, u8 v) {
        tick();
        // macros??
        #define writeOrRet(what) u8 &r = what; if (!write) return r; r=v
        if (addr <= 0x3FFF) {
            writeOrRet(Cartridge::ROM[addr]);
            printf("WARNING: Just wrote %d @ %04X\n", v, addr);
            /*exit(1);*/}
        else if (addr <= 0x7FFF) {
            writeOrRet(Cartridge::ROM[addr]);
            printf("WARNING: Just wrote %d @ %04X\n", v, addr);
            exit(1);}
        else if (addr <= 0x9FFF) {
            writeOrRet(PPU::VRAM[addr - 0x8000]);}
        else if (addr <= 0xBFFF) {
            printf("WARNING: The memory here (0x%04X) is not supposed to be accessed by MBC0 ROMS\n", addr);
            exit(1);}
        else if (addr <= 0xDFFF) {
            fprintf(stderr, "ERROR: WRAM is unimplemented (@ 0x%04X)\n", addr);
            exit(1);}
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
        // parsing by components:
        // https://gb-archive.github.io/salvage/decoding_gbz80_opcodes/Decoding%20Gamboy%20Z80%20Opcodes.html

        // static std::tuple<u8*,u8*,u8*,u8*,u8*,u8*,u16*,u8*> r = std::make_tuple(
        //    &registers.B, &registers.C, &registers.D,  &registers.E,
        //    &registers.H, &registers.L, &registers.HL, &registers.A
        // );

        // everything should be u8*, except for registers.HL
        void *r[] = {
           &registers.B, &registers.C, &registers.D,  &registers.E,
           &registers.H, &registers.L, &registers.HL, &registers.A
        };

        // static std::vector<std::any> r = {
        //    &registers.B, &registers.C, &registers.D,  &registers.E,
        //    &registers.H, &registers.L, &registers.HL, &registers.A
        // };

        static u16 *rp[4] = {
            &registers.BC, &registers.DE, &registers.HL, &registers.SP
        };

        static u16 *rp2[4] = {
            &registers.BC, &registers.DE, &registers.HL, &registers.AF
        };

        // to know the if the flags are set, we do i.e. !!(registers.F & FLAG_Z)
        // table "cc" (indexed 0)
        // NZ Z NC C
        // ^    ^
        // |    Not carry
        // Not zero

        static bool cc[4] = {
            (registers.F & FLAG_Z)!=0, (registers.F & FLAG_Z)==0,
            (registers.F & FLAG_C)!=0, (registers.F & FLAG_C)==0
        };

        // table alu
        static void (*alu[8])(u8) = {
            aluADDA, aluADCA, aluSUB, aluSBCA, aluAND, aluXOR, aluOR, aluCP
        };

        // table rot
        static void (*rot[8])(u8*) = {
            cbRLC, cbRRC, cbRL, cbRR, cbSLA, cbSRA, cbSWAP, cbSRL
        };

        enum { x = op>>6, y = (op>>3)&7, z = op&7, p = y>>1, q = y%2 };
        // printf("x: %d\t\ty: %d\t\tz: %d\t\tp: %d\t\tq: %d\n", x, y, z, p, q);
        unsigned cyc = 1; // machine cycles, T-states = machine cycles * 4
        switch (x) {
            // begin all x switch cases
            case 0: {
                // begin z switch-case
                switch (z) {
                    // Relative jumps
                    case 0: {
                        switch (y) {
                            /* NOP */
                            case 0: {nop();}  break;
                            /* LD NN */
                            case 1: {writeu16(ft(2), registers.SP); cyc+=4;} break;
                            /* STOP */
                            case 2: {stop();} break;
                            /* JR d */
                            JR8:
                            case 3: {jr(ft(1)); cyc+=2;} break;
                            case 4: case 5: case 6:
                            // TODO: Fix this because it is wrong
                            /* JR cc[y-4], d */
                            case 7: {cyc+=1; if (!cc[y-4]) return; goto JR8;} break;
                        }
                    } break;
                    // 16-bit [load immediate | add]
                    case 1: {
                        switch (q) {
                            /* LD rp[p], nn */
                            case 0: {load16bit(rp[(op>>4)/2], ft(2)); cyc+=2;} break;
                            /* ADD HL, rp[p]*/
                            case 1: {addhl(rp[(op>>4)/2]); cyc+=1;} break;
                        }
                    } break;
                    // Indirect loading
                    case 2: {
                        switch (q) {
                            case 0: {
                                if (q == 0) {WB(registers.BC, registers.A);}
                                else if (q == 1) {WB(registers.DE, registers.A);}
                                else if (q == 2) {
                                    WB(registers.HL, registers.A); registers.HL++;
                                }
                                else if (q == 3) {
                                    WB(registers.HL, registers.A); registers.HL--;
                                }
                                cyc+=1;
                            } break;
                            case 1: {
                                u8 rval;
                                if (q == 0) {rval = RB(registers.BC);}
                                else if (q == 1) {rval = RB(registers.DE);}
                                else if (q == 2) {
                                    rval = RB(registers.HL); registers.HL++;
                                }
                                else if (q == 3) {
                                    rval = RB(registers.HL); registers.HL--;
                                }
                                registers.A = rval;
                                cyc+=1;
                            } break;
                        }
                    } break;
                    // 16-bit INC/DEC
                    case 3: {
                        // from the manual: should not change any flags
                        switch (q) {
                            /* INC rp[p] */
                            case 0: {(*rp[p])++; cyc+=1;} break;
                            /* DEC rp[p] */
                            case 1: {(*rp[p])--; cyc+=1;} break;
                        }
                    } break;
                    // 8-bit INC
                    case 4: {
                        /* INC r[y] */
                        if (y!=6) {
                            // *(std::get<y>(r))++;
                            (*(u8*)r[y])++;
                            cyc+=0;
                        } else {
                            u8 t = RB(registers.HL);
                            WB(registers.HL, t++);
                            cyc+=2;
                        }
                    } break;
                    // 8-bit DEC
                    case 5: {
                        /* DEC r[y] */
                        if (y!=6) {
                            // *(std::get<y>(r))--;
                            (*(u8*)r[y])--;
                            cyc+=0;
                        } else {
                            u8 t = RB(registers.HL);
                            WB(registers.HL, t--);
                            cyc+=2;
                        }
                    } break;
                    // 8-bit load immediate
                    /* LD r[y], n */
                    case 6: {
                        if (y==6) {
                            // WB(*std::get<y>(r), ft(1)); cyc+=2;
                            WB(*(u8*)r[y], ft(1)); cyc+=2;
                        } else {
                            // load8bit(std::get<y>(r), ft(1)); cyc+=1;
                            load8bit((u8*)r[y], ft(1)); cyc+=1;
                        }
                    } break;
                    // operations on [accumulators | flags]
                    case 7: {
                        switch (y) {
                            case 0: {
                                /* RLCA */
                                // rl(std::get<7>(r), false); cyc+=0;
                                rl((u8*)r[7], false); cyc+=0;
                            } break;
                            case 1: {
                                /* RRCA */
                                // rr(std::get<7>(r), false); cyc+=0;
                                rr((u8*)r[7], false); cyc+=0;
                            } break;
                            case 2: {
                                /* RLA */
                                // rl(std::get<7>(r), true); cyc+=0;
                                rl((u8*)r[7], true); cyc+=0;
                            } break;
                            case 3: {
                                /* RRA */
                                // rr(std::get<7>(r), true); cyc+=0;
                                rr((u8*)r[7], true); cyc+=0;
                            } break;
                            case 4: {
                                /* DAA */
                                daa(); cyc+=0;
                            } break;
                            case 5: {
                                /* CPL */
                                registers.A = ~registers.A;
                                set_flag(FLAG_N, true);
                                set_flag(FLAG_H, true);
                                cyc+=0;
                            } break;
                            case 6: {
                                /* SCF */
                                set_flag(FLAG_N, false);
                                set_flag(FLAG_H, false);
                                set_flag(FLAG_C, true);
                                cyc+=0;
                            } break;
                            case 7: {
                                /* CCF */
                                set_flag(FLAG_N, false);
                                set_flag(FLAG_H, false);
                                set_flag(FLAG_C, !(registers.F & FLAG_C));
                                cyc+=0;
                            } break;
                        }
                    } break;
                }                              // end of z swich-case
            } break;                           // end of case x == 0
            case 1: {                          // begin of case x == 1
                if (z==6 && y==6) {
                    /* HALT */
                    if (interrupts.IF & interrupts.IE) registers.PC--;
                } else {
                    /* 8-bit loading (but with registers) */
                    if (y==6)
                        // WB(*std::get<y>(r), *std::get<z>(r));
                        WB(*(u16*)r[y], *(u8*)r[z]);
                    else if (z==6)
                        // load8bit(std::get<y>(r), RB(*std::get<z>(r)));
                        load8bit((u8*)r[y], RB(*(u16*)r[z]));
                    else
                        // load8bit(std::get<y>(r), *std::get<z>(r));
                        load8bit((u8*)r[y], *(u8*)r[z]);
                    cyc+=(z==6 || y==6);
                }
            } break;                           // end of case x == 1
            case 2: {
                /* Arithmetic / logic operations -- ALU */
                if (z==6) {
                    alu[y](RB(*(u16*)r[z]));
                    cyc+=1;
                } else {
                    alu[y](*(u8*)r[z]);
                    cyc+=0;
                }
            } break;                           // end of case x == 2
            case 3: {
                switch (z) {
                    /* Conditional return, mem-mapped register loads and stack operations */
                    case 0: {
                        switch (y) {
                            // RET cc[y]
                            case 0: case 1: case 2:
                            case 3: {if (cc[y]) {ret(); cyc+=4;} cyc+=1;} break;
                            // LDH (n), A
                            case 4: {WB(0xFF00+ft(1), registers.A); cyc+=2;} break;
                            // ADD SP, n
                            case 5: {
                                int16_t s_v = (int16_t) (int8_t) ft(1),
                                        res = (u16) ((int16_t)registers.SP + s_v);
                                set_flag(FLAG_Z, false);
                                set_flag(FLAG_N, false);
                                set_flag(FLAG_H, (res & 0xF)  < (registers.SP & 0xF));
                                set_flag(FLAG_C, (res & 0xFF) < (registers.SP & 0xFF));
                                registers.SP = res;
                                cyc+=3;
                            } break;
                            // LDH A, (n)
                            case 6: {registers.A  = RB(0xFF00+ft(1)); cyc+=2;} break;
                            /* LD HL, SP +d */
                            case 7: {
                                int16_t s_v = (int16_t) (int8_t) ft(1),
                                          r = (u16) (int16_t) registers.SP + s_v;
                                set_flag(FLAG_Z, false);
                                set_flag(FLAG_H, (r & 0xF) < (registers.SP & 0xF)); // set if carry from bit 11
                                set_flag(FLAG_C, (r & 0xFF) < (registers.SP & 0xFF)); // set if carry from bit 15
                                set_flag(FLAG_N, false);
                                registers.HL = r;
                                cyc+=2;
                            } break;
                        }
                    } break;
                    case 1: {
                        if (q == 1) {
                            if (p == 0) {
                                /* RET */
                                ret();
                                cyc+=3;
                                break;
                            } else if (p == 1) {
                                /* RETI */
                                interrupts.IME = 1;
                                ret();
                                cyc+=3;
                            } else if (p == 2) {
                                /* JP HL */
                                registers.PC = registers.HL;
                                cyc+=0;
                            } else {
                                /* LD SP, HL */
                                registers.SP = registers.HL;
                                cyc+=1;
                            }
                        } else {
                            /* POP rp2[p] */
                            u16 lo = (u16)RB(registers.SP++);
                            u16 hi = (u16)RB(registers.SP++);
                            *rp2[p] = (hi << 0xF) | lo;
                            cyc+=2;
                        }
                    } break;
                    case 2: {
                        switch (y) {
                            case 0: case 1: case 2:
                            case 3: {
                                /* JP cc[y], nn */
                                if (cc[y]) {
                                    jp(ft(2));
                                    cyc+=1;
                                }
                                cyc+=2;
                            } break;
                            case 4: {
                                /* LD (0xFF00+C), A */
                                WB(0xFF00 + registers.C, registers.A);
                                cyc+=1;
                            } break;
                            case 5: {
                                /* LD (nn), A */
                                WB(ft(2), registers.A);
                                cyc+=3;
                            } break;
                            case 6: {
                                /* LD A, (0xFF00+C) */
                                registers.A = RB(0xFF00 + registers.C);
                                cyc+=1;
                            } break;
                            case 7: {
                                /* LD A, (nn) */
                                registers.A = RB(ft(2));
                                cyc+=3;
                            } break;
                        }
                    } break;;
                    case 3: {
                        switch (y) {
                            /* JP nn */
                            case 0: {
                                jp(ft(2)); cyc+=3;
                            } break;
                            /* (CB prefix) */
                            case 1: {
                                const u8 displacement = ft(1);
                                const u8 dx = displacement>>6,
                                             dy = (displacement>>3)&7,
                                             dz = displacement&7,
                                             dp = y>>1,
                                             dq = y%2;
                                switch (dx) {
                                    case 0: {
                                        if (dz==6) {
                                            u8 t = RB(*(u16*)r[dz]); rot[dy](&t);
                                            WB(*(u16*)r[dz], t); cyc+=3;
                                        } else {rot[dy]((u8*)r[dz]); cyc+=1;}
                                    } break;
                                    case 1: {
                                        if (dz==6) {
                                            cbBIT(RB(*(u16*)r[dz]), dy); cyc+=3;
                                        } else {cbBIT(*(u8*)r[dz], dy); cyc+=1;}
                                    } break;
                                    case 2: {
                                        if (dz==6) {
                                            u8 t = RB(*(u16*)r[dz]); cbRES(&t, dy);
                                            WB(*(u16*)r[dz], t); cyc+=3;
                                        } else {cbRES((u8*)r[dz], dy); cyc+=1;}
                                    } break;
                                    case 3: {
                                        if (dz==6) {
                                            u8 t = RB(*(u16*)r[dz]); cbSET(&t, dy);
                                            WB(*(u16*)r[dz], t); cyc+=3;
                                        } else {cbSET((u8*)r[dz], dy); cyc+=1;}
                                    } break;
                                }
                            } break;
                            case 6: {interrupts.IME = false; cyc+=0;} break;
                            case 7: {interrupts.IME = true; cyc+=0;} break;
                        }
                    } break;
                    case 4: {
                        switch (y) {
                            case 0: case 1: case 2:
                            case 3: {
                                if (cc[y]) {call(ft(2)); cyc+=5;}
                                else {cyc+=2;}
                            } break;
                        }
                    };
                    case 5: {
                        switch (q) {
                            case 0: {
                                u16 t = *rp2[p];
                                WB(--(registers.SP), (u8)(t >> 8));
                                WB(--(registers.SP), t & 0xFF);
                                cyc+=3;
                            } break;
                            case 1: {
                                if (p == 0) {call(ft(2)); cyc+=5;}
                            } break;
                        }
                    } break;
                    case 6: {alu[y](ft(1)); cyc+=1;} break;
                    case 7: {
                        WB(--(registers.SP), registers.PC >> 8);
                        WB(--(registers.SP), registers.PC & 0xFF);
                        cyc+=3;
                    } break;
                }                              // end of z switch-case
            } break;                           // end of case x == 3
            // end of all x switch-cases
            default: {
                printf("ERROR: opcode 0x%02X is unimplemented\n", op);
                printf("TRACE: was called @ addr 0x%04X\n", registers.PC-1);
                exit(1);
            }
        }
    }
    // this would be the ``loop''
    void Op() {
        unsigned op = RB(registers.PC++);
        printf("PC: 0x%04X\t\tExecuting: 0x%02X\n", registers.PC-1, op);

        // Today I learned about the paste operator a.k.a. ## in c++
        #define c(n) Ins<0x##n>, Ins<0x##n+1>,
        #define o(n) c(n)c(n+2)c(n+4)c(n+6)
        static void (*i[0x108])() = {
            o(00)o(08)o(10)o(18)o(20)o(28)o(30)o(38)
            o(40)o(48)o(50)o(58)o(60)o(68)o(70)o(78)
            o(80)o(88)o(90)o(98)o(A0)o(A8)o(B0)o(B8)
            o(C0)o(C8)o(D0)o(D8)o(E0)o(E8)o(F0)o(F8)
        };
        #undef o
        #undef c

        i[op]();
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

    // printf("pc: %04X\n", CPU::registers.PC);

    // for (int i = CPU::registers.PC; i <= 0x8000; ++i) {
    //     printf("0x%02X\t0x%04X\n", Cartridge::ROM[i], i);
    // }

    for (;;) {
        CPU::Op();
    }
    return(0);
}
