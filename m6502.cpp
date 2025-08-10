/* Ivan Naranjo Ortega
 * Based on Obelisk's documentation:
 *   http://www.6502.org/users/obelisk/index.html
 */

/* Basic structure information:
 *   - 8 bit CPU
 *   - 64 Kb of memory
 *   - 16 bit address bus
 *   - LSB
 *   - First 256 byte page (0x0000 - 0x00FF) is the zero page,
 *     and is the focus of a number of special addressing modes
 *     that result in shorter instructions or allow indirect
 *     access to the memory.
 *   - Second 256 byte page (0x0100 - 0x01FF) is reserved for
 *     the system stack.
 *   - The very last 6 bytes (0xFFFA - 0xFFFF) are reserved
 *     locations. Contains
 *       - 0xFFFA/B, non-maskable interrupt handler.
 *       - 0xFFFC/D, power on reset location.
 *       - 0xFFFE/F, BRK/interrupt request handler respectively.
 */

/* For degugging */
#include <iostream>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <string>
#include <chrono>

namespace m6502{
  using byte = unsigned char;
  using word = unsigned short;

  using u32 = unsigned int;

  u32 cycles;

  struct StatusFlags
  {
    byte C : 1;       // 0: Carry Flag
    byte Z : 1;       // 1: Zero Flag
    byte I : 1;       // 2: Interrupt Disable
    byte D : 1;       // 3: Decimal Mode
    byte B : 1;       // 4: Break Command
    byte Unused : 1;  // 5: Unused
    byte V : 1;       // 6: Overflow Flag
    byte N : 1;       // 7: Negative Flag
  };

  enum Opcode : byte {
    LDA_IMMEDIATE = 0xA9,
    LDA_ZEROPAGE  = 0xA5,
    LDA_ZEROPAGEX = 0xB5,
    LDA_ABSOLUTE  = 0xAD,
    LDA_ABSOLUTEX = 0xBD,
  };

  enum class LogLevel : int {
    Error   = 0,
    Warning = 1,
    Info    = 2
  };  
  inline LogLevel compilationLogLevel = LogLevel::Info; // TODO: Change this during compilation or execution
  inline auto start_time = std::chrono::steady_clock::now();
  inline void log(LogLevel level, const char* format, ...) {
    if ((int)level <= (int)compilationLogLevel) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>
        (now - start_time).count();

      std::cout << "[" << elapsed << "ns]";

      switch (level) {
        case LogLevel::Error:   std::cout << "[ERRO] "; break;
        case LogLevel::Warning: std::cout << "[WARN] "; break;
        case LogLevel::Info:    std::cout << "[INFO] "; break;
      }


      static constexpr size_t BUFFER_SIZE = 1024;
      char buffer[BUFFER_SIZE];

      va_list args;
      va_start(args, format);
      vsnprintf(buffer, BUFFER_SIZE, format, args);
      va_end(args);

      std::cout << buffer;
    }
  }

  struct MEM
  {
    static constexpr u32 MAX_MEM = 1024 * 64;  // 64 KB
    byte data[MAX_MEM];

    byte operator[](word addr) const { ++cycles; return data [addr]; }
    byte& operator[](word addr) { ++cycles; return data[addr]; }

    void clear() {
      for (u32 i = 0; i < MAX_MEM; ++i)
        data[i] = 0;
    }
    
    void setResetVector() {
      data[0xFFFC] = 0x00;
      data[0xFFFD] = 0x80;
    }
  };

  struct CPU
  {
    word PC;    // Program Counter
    byte SP;    // Stack Pointer

    byte 
      A,        // Accumulator
      X,        // Index Register X
      Y         // Index Register Y
    ;

    union
    {
      byte PS;            // Processor Status
      StatusFlags Flag;   // Individual flags
    } P;


    void reset(MEM& mem) {
      mem.clear();
      mem.setResetVector();
      PC = mem[0xFFFC] | (mem[0xFFFD] << 8);
      SP = 0xFD;
      A = Y = X = 0;
      P.PS = 0b00100100;
      cycles = 0;

      log(LogLevel::Info,"CPU resetted\n");
    }

    byte fetchByte(const MEM& mem) {
      log(LogLevel::Info, "Fetched 0x%02x from mem[0x%04x]\n", mem[PC],PC); 
      --cycles; // Remove cycle counted in log call 

      return mem[PC++];
    }

    void setZN(byte value){
      P.Flag.Z = (value == 0);
      P.Flag.N = (value & 0b10000000) != 0;
    }

    void execInstruction(const MEM& mem) {
      byte opcode = fetchByte(mem);
      
      switch(opcode) {
        case LDA_IMMEDIATE: {
          log(LogLevel::Info,"LDA Immediate\n");
          byte value = fetchByte(mem);
          A = value;
          setZN(A);
        } break;

        case LDA_ZEROPAGE: {
          log(LogLevel::Info,"LDA ZP\n");
          byte zpAddr = fetchByte(mem);
          A = mem[zpAddr];
          setZN(A);
        } break;

        case LDA_ZEROPAGEX: {
          log(LogLevel::Info,"LDA ZP X\n");
          byte zpAddr = fetchByte(mem);
          zpAddr = (zpAddr + X) & 0xFF;
          A = mem[zpAddr];
          setZN(A);
        } break;

        case LDA_ABSOLUTE: {
          log(LogLevel::Info,"LDA Absolute\n");
          byte lowAddr = fetchByte(mem);
          byte highAddr = fetchByte(mem);
          word addr = lowAddr | (highAddr << 8);
          A = mem[addr];
          setZN(A);
        } break;

        case LDA_ABSOLUTEX: {
          log(LogLevel::Info,"LDA Absolute X\n");
          byte lowAddr = fetchByte(mem);
          byte highAddr = fetchByte(mem);
          word addr = (lowAddr | (highAddr << 8)) + X;
          A = mem[addr];
          setZN(A);
        } break;

        default: {
          log(LogLevel::Error,"Unknown opcode: 0x%02X\n", opcode);
        } break;
      }
    }   
  };
}

int main()
{
  m6502::MEM mem;
  m6502::CPU cpu;

  cpu.reset(mem);
  /* EMBEDDED CODE */
  /* Code 1: LDA Immediate */
  /*
  mem[0x8000] = 0xA9; // Instruction
  mem[0x8001] = 0x55; // Byte to put in A
  */
  /* Code 2: LDA Zero Page */
  /*
  mem[0x8000] = 0xA5; // Instruction
  mem[0x8001] = 0x02; // Addr in Zero Page
  mem[0x0002] = 0x15; // Byte to put in A
  */
  // Code 3: LDA Zero Page X
  /*
  cpu.X = 0x10; //LDX not implemented yet
  mem[0x8000] = 0xB5; // Instruction
  mem[0x8001] = 0xFB; // Addr at ZP to be added with rX
  mem[0x000B] = 0x12; // Byte to put in A
  */
  // Code 4: LDA Absolute
  /*
  mem[0x8000] = 0xAD; // Instruction
  mem[0x8001] = 0x02; // Addr 0x0102
  mem[0x8002] = 0x01;
  mem[0x0102] = 0xAB; //Byte to put in A
  */
  // Code 5: LDA Absolute X
  cpu.X = 0x10; //LDX not implemented yet
  mem[0x8000] = 0xBD; // Instruction
  mem[0x8001] = 0x02; // Addr 0x0102
  mem[0x8002] = 0x01;
  mem[0x0112] = 0xAB; //Byte to put in A
  /* END OF EMBEDDED CODE */ 

  m6502::cycles = 0;
  cpu.execInstruction(mem);

  m6502::log(m6502::LogLevel::Info,"Final results:\n");
  m6502::log(m6502::LogLevel::Info,"  PC -> 0x%04X\n",cpu.PC);
  m6502::log(m6502::LogLevel::Info,"  Cy -> %u\n",m6502::cycles);
  m6502::log(m6502::LogLevel::Info,"  Ac -> 0x%02X\n",cpu.A);
  m6502::log(m6502::LogLevel::Info,"  rX -> 0x%02X\n",cpu.X);
  m6502::log(m6502::LogLevel::Info,"  rY -> 0x%02X\n",cpu.Y);
}
