

    void Emulator::run(std::string rom_path)
    {
        m_cartridge.loadFromFile(rom_path)

        m_mapper = Mapper::createMapper(info from m_cartridge ...)

        m_bus.       setMapper(m_mapper.get())
        m_pictureBus.setMapper(m_mapper.get())

        m_cpu.reset();
        m_ppu.reset();

        m_window.        create(....)
        m_emulatorScreen.create(....);

        m_cycleTimer = std::chrono::high_resolution_clock::now();
        m_elapsedTime = m_cycleTimer - m_cycleTimer;

        sf::Event event;
        bool focus = true, pause = false;
        while (m_window.isOpen())
        {
            while (m_window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed ||
                (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape))
                {
                    m_window.close(); return;
                }
                else if (event.type == sf::Event::GainedFocus)
                {
                    focus = true;
                    m_cycleTimer = std::chrono::high_resolution_clock::now();
                }
                else if (event.type == sf::Event::LostFocus)
                    focus = false;
                else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::F2)
                {
                    pause = !pause;
                    if (!pause)
                    {
                        m_cycleTimer = std::chrono::high_resolution_clock::now();
                    }
                }
                else if (pause && event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::F3)
                {
                    for (int i = 0; i < 29781; ++i) //Around one frame
                    {
                        //PPU
                        m_ppu.step(); m_ppu.step(); m_ppu.step();
                        //CPU
                        m_cpu.step();
                    }
                }
            }

            if (focus && !pause)
            {
                m_elapsedTime += std::chrono::high_resolution_clock::now() - m_cycleTimer;
                m_cycleTimer = std::chrono::high_resolution_clock::now();

                while (m_elapsedTime > m_cpuCycleDuration)
                {
                    //PPU
                    m_ppu.step(); m_ppu.step(); m_ppu.step();
                    //CPU
                    m_cpu.step();

                    m_elapsedTime -= m_cpuCycleDuration;
                }

                m_window.draw(m_emulatorScreen);
                m_window.display();
            }
            else
            {
                sf::sleep(sf::milliseconds(1000/60));
            }
        }
    }

















namespace sn
{

    class CPU
    {
        public:

            CPU(MainBus &mem);

            void step();
            void reset();
            void reset(Address start_addr);
            void log();

            Address getPC() { return r_PC; }
            void skipDMACycles();

            void interrupt(InterruptType type);

        private:
            void interruptSequence(InterruptType type);

            //Instructions are split into five sets to make decoding easier.
            //These functions return true if they succeed
            bool executeImplied(Byte opcode);
            bool executeBranch(Byte opcode);
            bool executeType0(Byte opcode);
            bool executeType1(Byte opcode);
            bool executeType2(Byte opcode);

            Address readAddress(Address addr);

            void pushStack(Byte value);
            Byte pullStack();

            //If a and b are in different pages, increases the m_SkipCycles by inc
            void setPageCrossed(Address a, Address b, int inc = 1);
            void setZN(Byte value);

            int m_skipCycles;
            int m_cycles;

            //Registers
            Address r_PC;
            Byte r_SP;
            Byte r_A;
            Byte r_X;
            Byte r_Y;

            //Status flags.
            //Is storing them in one byte better ?
            bool f_C;
            bool f_Z;
            bool f_I;
            bool f_D;
            bool f_V;
            bool f_N;

            bool m_pendingNMI; // non maskable interrupt
            bool m_pendingIRQ;

            MainBus &m_bus;
    };

};
#endif // CPU_H





















#include "CPU.h"
#include "CPUOpcodes.h"
#include "Log.h"
#include <iomanip>

namespace sn
{
    CPU::CPU(MainBus &mem) :
        m_pendingNMI(false),
        m_pendingIRQ(false),
        m_bus(mem)
    {}

    void CPU::reset()
    {
        reset(readAddress(ResetVector));
    }

    void CPU::reset(Address start_addr)
    {
        m_skipCycles = m_cycles = 0;
        r_A = r_X = r_Y = 0;
        f_I = true;
        f_C = f_D = f_N = f_V = f_Z = false;
        r_PC = start_addr;
        r_SP = 0xfd; //documented startup state
    }

    void CPU::interrupt(InterruptType type)
    {
        switch (type)
        {
        case InterruptType::NMI:
            m_pendingNMI = true;
            break;

        case InterruptType::IRQ:
            m_pendingIRQ = true;
            break;

        default:
            break;
        }
    }

    void CPU::interruptSequence(InterruptType type)
    {
        if (f_I && type != NMI && type != BRK_)
            return;

        if (type == BRK_) //Add one if BRK, a quirk of 6502
            ++r_PC;

        pushStack(r_PC >> 8);
        pushStack(r_PC);

        Byte flags = f_N << 7 |
                     f_V << 6 |
                       1 << 5 | //unused bit, supposed to be always 1
          (type == BRK_) << 4 | //B flag set if BRK
                     f_D << 3 |
                     f_I << 2 |
                     f_Z << 1 |
                     f_C;
        pushStack(flags);

        f_I = true;

        switch (type)
        {
            case IRQ:
            case BRK_:
                r_PC = readAddress(IRQVector);
                break;
            case NMI:
                r_PC = readAddress(NMIVector);
                break;
        }

        // Interrupt sequence takes 7, but one cycle was actually spent on this.
        // So skip 6
        m_skipCycles += 6;
    }

    void CPU::pushStack(Byte value)
    {
        m_bus.write(0x100 | r_SP, value);
        --r_SP; //Hardware stacks grow downward!
    }

    Byte CPU::pullStack()
    {
        return m_bus.read(0x100 | ++r_SP);
    }

    void CPU::setZN(Byte value)
    {
        f_Z = !value;
        f_N = value & 0x80;
    }

    void CPU::setPageCrossed(Address a, Address b, int inc)
    {
        //Page is determined by the high byte
        if ((a & 0xff00) != (b & 0xff00))
            m_skipCycles += inc;
    }

    void CPU::skipDMACycles()
    {
        m_skipCycles += 513; //256 read + 256 write + 1 dummy read
        m_skipCycles += (m_cycles & 1); //+1 if on odd cycle
    }

    void CPU::step()
    {
        ++m_cycles;
        if (m_skipCycles-- > 1) return;
        m_skipCycles = 0;

        if      (m_pendingNMI) { do sth; return;  }
        else if (m_pendingIRQ) { do sth; return;  }

        Byte opcode = m_bus.read(r_PC++);

        auto CycleLength = OperationCycles[opcode];

        if (CycleLength && (executeImplied(opcode) || executeBranch(opcode) ||
                        executeType1(opcode) || executeType2(opcode) || executeType0(opcode)))
            m_skipCycles += CycleLength;
    }

    bool CPU::executeImplied(Byte opcode)
    {
        switch (static_cast<OperationImplied>(opcode))
        {
            case NOP:
                break;
            case BRK:
                interruptSequence(BRK_);
                break;
            // operations on flags, regs, stack
            case JSR:
            case RTS:
            case RTI:
            case JMP:
            case JMPI:
            case PHP:
            case PLP:
            case PHA:
            case PLA:
            case DEY:
            case DEX:
            case TAY:
            case INY:
            case INX:
            case CLC:
            case SEC:
            case CLI:
            case SEI:
            case CLD:
            case SED:
            case TYA:
            case CLV:
            case TXA:
            case TXS:
            case TAX:
            case TSX:
        };
        return true;
    }

    bool CPU::executeBranch(Byte opcode)
    {
        if ((opcode & BranchInstructionMask) == BranchInstructionMaskResult)
        {
            bool branch = opcode & BranchConditionMask;

            switch (opcode >> BranchOnFlagShift)
            {
                // manipulate on 'branch' variable based on cpu flags
                case Negative:
                case Overflow:
                case Carry:
                case Zero:
            }

            if (branch)
            {
                int8_t offset = m_bus.read(r_PC++);
                ++m_skipCycles;
                auto newPC = static_cast<Address>(r_PC + offset);
                setPageCrossed(r_PC, newPC, 2);
                r_PC = newPC;
            }
            else
                ++r_PC;
            return true;
        }
        return false;
    }

    bool CPU::executeType1(Byte opcode)
    {
        if ((opcode & InstructionModeMask) == 0x1)
        {
            Address location = 0; //Location of the operand, could be in RAM
            auto op = static_cast<Operation1>((opcode & OperationMask) >> OperationShift);
            switch (static_cast<AddrMode1>(
                    (opcode & AddrModeMask) >> AddrModeShift))
            {
                case IndexedIndirectX:
                    {
                        Byte zero_addr = r_X + m_bus.read(r_PC++);
                        //Addresses wrap in zero page mode, thus pass through a mask
                        location = m_bus.read(zero_addr & 0xff) | m_bus.read((zero_addr + 1) & 0xff) << 8;
                    }
                    break;
                case ZeroPage:
                    location = m_bus.read(r_PC++);
                    break;
                case Immediate:
                    location = r_PC++;
                    break;
                case Absolute:
                    location = readAddress(r_PC);
                    r_PC += 2;
                    break;
                case IndirectY:
                    {
                        Byte zero_addr = m_bus.read(r_PC++);
                        location = m_bus.read(zero_addr & 0xff) | m_bus.read((zero_addr + 1) & 0xff) << 8;
                        if (op != STA)
                            setPageCrossed(location, location + r_Y);
                        location += r_Y;
                    }
                    break;
                case IndexedX:
                    // Address wraps around in the zero page
                    location = (m_bus.read(r_PC++) + r_X) & 0xff;
                    break;
                case AbsoluteY:
                    location = readAddress(r_PC);
                    r_PC += 2;
                    if (op != STA)
                        setPageCrossed(location, location + r_Y);
                    location += r_Y;
                    break;
                case AbsoluteX:
                    location = readAddress(r_PC);
                    r_PC += 2;
                    if (op != STA)
                        setPageCrossed(location, location + r_X);
                    location += r_X;
                    break;
                default:
                    return false;
            }

            switch (op)
            {
                case ORA:
                    r_A |= m_bus.read(location);
                    setZN(r_A);
                    break;
                case AND:
                    r_A &= m_bus.read(location);
                    setZN(r_A);
                    break;
                case EOR:
                    r_A ^= m_bus.read(location);
                    setZN(r_A);
                    break;
                case ADC:
                    {
                        Byte operand = m_bus.read(location);
                        std::uint16_t sum = r_A + operand + f_C;
                        //Carry forward or UNSIGNED overflow
                        f_C = sum & 0x100;
                        //SIGNED overflow, would only happen if the sign of sum is
                        //different from BOTH the operands
                        f_V = (r_A ^ sum) & (operand ^ sum) & 0x80;
                        r_A = static_cast<Byte>(sum);
                        setZN(r_A);
                    }
                    break;
                case STA:
                    m_bus.write(location, r_A);
                    break;
                case LDA:
                    r_A = m_bus.read(location);
                    setZN(r_A);
                    break;
                case SBC:
                    {
                        //High carry means "no borrow", thus negate and subtract
                        std::uint16_t subtrahend = m_bus.read(location),
                                 diff = r_A - subtrahend - !f_C;
                        //if the ninth bit is 1, the resulting number is negative => borrow => low carry
                        f_C = !(diff & 0x100);
                        //Same as ADC, except instead of the subtrahend,
                        //substitute with it's one complement
                        f_V = (r_A ^ diff) & (~subtrahend ^ diff) & 0x80;
                        r_A = diff;
                        setZN(diff);
                    }
                    break;
                case CMP:
                    {
                        std::uint16_t diff = r_A - m_bus.read(location);
                        f_C = !(diff & 0x100);
                        setZN(diff);
                    }
                    break;
                default:
                    return false;
            }
            return true;
        }
        return false;
    }

    bool CPU::executeType2(Byte opcode)
    {
        if ((opcode & InstructionModeMask) == 2)
        {
            Address location = 0;
            auto op = static_cast<Operation2>((opcode & OperationMask) >> OperationShift);
            auto addr_mode =
                    static_cast<AddrMode2>((opcode & AddrModeMask) >> AddrModeShift);
            switch (addr_mode)
            {
                case Immediate_:
                    location = r_PC++;
                    break;
                case ZeroPage_:
                    location = m_bus.read(r_PC++);
                    break;
                case Accumulator:
                    break;
                case Absolute_:
                    location = readAddress(r_PC);
                    r_PC += 2;
                    break;
                case Indexed:
                    {
                        location = m_bus.read(r_PC++);
                        Byte index;
                        if (op == LDX || op == STX)
                            index = r_Y;
                        else
                            index = r_X;
                        //The mask wraps address around zero page
                        location = (location + index) & 0xff;
                    }
                    break;
                case AbsoluteIndexed:
                    {
                        location = readAddress(r_PC);
                        r_PC += 2;
                        Byte index;
                        if (op == LDX || op == STX)
                            index = r_Y;
                        else
                            index = r_X;
                        setPageCrossed(location, location + index);
                        location += index;
                    }
                    break;
                default:
                    return false;
            }

            std::uint16_t operand = 0;
            switch (op)
            {
                case ASL:
                case ROL:
                    if (addr_mode == Accumulator)
                    {
                        auto prev_C = f_C;
                        f_C = r_A & 0x80;
                        r_A <<= 1;
                        //If Rotating, set the bit-0 to the the previous carry
                        r_A = r_A | (prev_C && (op == ROL));
                        setZN(r_A);
                    }
                    else
                    {
                        auto prev_C = f_C;
                        operand = m_bus.read(location);
                        f_C = operand & 0x80;
                        operand = operand << 1 | (prev_C && (op == ROL));
                        setZN(operand);
                        m_bus.write(location, operand);
                    }
                    break;
                case LSR:
                case ROR:
                    if (addr_mode == Accumulator)
                    {
                        auto prev_C = f_C;
                        f_C = r_A & 1;
                        r_A >>= 1;
                        //If Rotating, set the bit-7 to the previous carry
                        r_A = r_A | (prev_C && (op == ROR)) << 7;
                        setZN(r_A);
                    }
                    else
                    {
                        auto prev_C = f_C;
                        operand = m_bus.read(location);
                        f_C = operand & 1;
                        operand = operand >> 1 | (prev_C && (op == ROR)) << 7;
                        setZN(operand);
                        m_bus.write(location, operand);
                    }
                    break;
                case STX:
                    m_bus.write(location, r_X);
                    break;
                case LDX:
                    r_X = m_bus.read(location);
                    setZN(r_X);
                    break;
                case DEC:
                    {
                        auto tmp = m_bus.read(location) - 1;
                        setZN(tmp);
                        m_bus.write(location, tmp);
                    }
                    break;
                case INC:
                    {
                        auto tmp = m_bus.read(location) + 1;
                        setZN(tmp);
                        m_bus.write(location, tmp);
                    }
                    break;
                default:
                    return false;
            }
            return true;
        }
        return false;
    }

    bool CPU::executeType0(Byte opcode)
    {
        if ((opcode & InstructionModeMask) == 0x0)
        {
            Address location = 0;
            switch (static_cast<AddrMode2>((opcode & AddrModeMask) >> AddrModeShift))
            {
                case Immediate_:
                    location = r_PC++;
                    break;
                case ZeroPage_:
                    location = m_bus.read(r_PC++);
                    break;
                case Absolute_:
                    location = readAddress(r_PC);
                    r_PC += 2;
                    break;
                case Indexed:
                    // Address wraps around in the zero page
                    location = (m_bus.read(r_PC++) + r_X) & 0xff;
                    break;
                case AbsoluteIndexed:
                    location = readAddress(r_PC);
                    r_PC += 2;
                    setPageCrossed(location, location + r_X);
                    location += r_X;
                    break;
                default:
                    return false;
            }
            std::uint16_t operand = 0;
            switch (static_cast<Operation0>((opcode & OperationMask) >> OperationShift))
            {
                case BIT:
                    operand = m_bus.read(location);
                    f_Z = !(r_A & operand);
                    f_V = operand & 0x40;
                    f_N = operand & 0x80;
                    break;
                case STY:
                    m_bus.write(location, r_Y);
                    break;
                case LDY:
                    r_Y = m_bus.read(location);
                    setZN(r_Y);
                    break;
                case CPY:
                    {
                        std::uint16_t diff = r_Y - m_bus.read(location);
                        f_C = !(diff & 0x100);
                        setZN(diff);
                    }
                    break;
                case CPX:
                    {
                        std::uint16_t diff = r_X - m_bus.read(location);
                        f_C = !(diff & 0x100);
                        setZN(diff);
                    }
                    break;
                default:
                    return false;
            }

            return true;
        }
        return false;
    }

    Address CPU::readAddress(Address addr)
    {
        return m_bus.read(addr) | m_bus.read(addr + 1) << 8;
    }

};





// SFML
// https://www.sfml-dev.org/tutorials/2.6/

#include <SFML/Graphics.hpp>


    class VirtualScreen : public sf::Drawable
    {
    public:
        void create (unsigned int width, unsigned int height, float pixel_size, sf::Color color);
        void setPixel (std::size_t x, std::size_t y, sf::Color color);

    private:
        void draw(sf::RenderTarget& target, sf::RenderStates states) const;

        sf::Vector2u m_screenSize;
        float m_pixelSize; //virtual pixel size in real pixels
        sf::VertexArray m_vertices;
    };


    void VirtualScreen::create(unsigned int w, unsigned int h, float pixel_size, sf::Color color)
    {
        m_vertices.resize(w * h * 6);
        m_screenSize = {w, h};
        m_vertices.setPrimitiveType(sf::Triangles);
        m_pixelSize = pixel_size;
        for (std::size_t x = 0; x < w; ++x)
        {
            for (std::size_t y = 0; y < h; ++y)
            {
                auto index = (x * m_screenSize.y + y) * 6;
                sf::Vector2f coord2d (x * m_pixelSize, y * m_pixelSize);

                //Triangle-1
                //top-left
                m_vertices[index].position = coord2d;
                m_vertices[index].color    = color;

                //top-right
                m_vertices[index + 1].position = coord2d + sf::Vector2f{m_pixelSize, 0};
                m_vertices[index + 1].color = color;

                //bottom-right
                m_vertices[index + 2].position = coord2d + sf::Vector2f{m_pixelSize, m_pixelSize};
                m_vertices[index + 2].color = color;

                //Triangle-2
                //bottom-right
                m_vertices[index + 3].position = coord2d + sf::Vector2f{m_pixelSize, m_pixelSize};
                m_vertices[index + 3].color = color;

                //bottom-left
                m_vertices[index + 4].position = coord2d + sf::Vector2f{0, m_pixelSize};
                m_vertices[index + 4].color = color;

                //top-left
                m_vertices[index + 5].position = coord2d;
                m_vertices[index + 5].color    = color;
            }
        }
    }

    void VirtualScreen::setPixel(std::size_t x, std::size_t y, sf::Color color)
    {
        auto index = (x * m_screenSize.y + y) * 6;
        if (index >= m_vertices.getVertexCount())
            return;

        sf::Vector2f coord2d (x * m_pixelSize, y * m_pixelSize);

        //Triangle-1
        //top-left
        m_vertices[index].color    = color;

        //top-right
        m_vertices[index + 1].color = color;

        //bottom-right
        m_vertices[index + 2].color = color;

        //Triangle-2
        //bottom-right
        m_vertices[index + 3].color = color;

        //bottom-left
        m_vertices[index + 4].color = color;

        //top-left
        m_vertices[index + 5].color    = color;
    }

    void VirtualScreen::draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        target.draw(m_vertices, states);
    }




    class Emulator
    {
        sf::RenderWindow m_window;
        VirtualScreen m_emulatorScreen;
    };



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

        m_window.create(sf::VideoMode(NESVideoWidth * m_screenScale, NESVideoHeight * m_screenScale),
                        "SimpleNES", sf::Style::Titlebar | sf::Style::Close | sf::Style::Resize);
        m_window.setVerticalSyncEnabled(true);


        m_emulatorScreen.create(NESVideoWidth, NESVideoHeight, m_screenScale, sf::Color::White);


        while (m_window.isOpen())
        {
            while (m_window.pollEvent(event))
            {
                // .....
            }

            if (focus && !pause)
            {
                // ...
                m_window.draw(m_emulatorScreen);
                m_window.display();
            }
        }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////





















// ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????

                // m_cpuCycleDuration set in ctor of Emulator - m_cpuCycleDuration(std::chrono::nanoseconds(559))
                // 559 probably optimal or something. look at 'pong' games - if i increase m_cpuCycleDuration the ball moves
                // more slowly, if i decrease it, the ball speeds up.


                // Emulator.h
                // could play with this more using cout etc, but hard to print those chromo types

                using TimePoint = std::chrono::high_resolution_clock::time_point;

                TimePoint                                      m_cycleTimer;
                std::chrono::high_resolution_clock::duration   m_elapsedTime;
                std::chrono::nanoseconds                       m_cpuCycleDuration;



                // Emulator.cpp

                // guess every time we reach here, we set m_elapsedTime to same value by doing (m_ElapsedTime += now - m_cycleTimer)
                m_elapsedTime += std::chrono::high_resolution_clock::now() - m_cycleTimer;
                m_cycleTimer = std::chrono::high_resolution_clock::now();

                while (m_elapsedTime > m_cpuCycleDuration)
                {
                    //PPU
                    m_ppu.step(); m_ppu.step(); m_ppu.step();
                    //CPU
                    m_cpu.step();

                    m_elapsedTime -= m_cpuCycleDuration;
                }

                m_window.draw(m_emulatorScreen);
                m_window.display();


// ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
// ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
// ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????


// PPU pipeline



    void PPU::step()
    {

        switch (m_pipelineState)
        {
            case PreRender:
                if (m_cycle == 1)
                    m_vblank = m_sprZeroHit = false;
                else if (m_cycle == ScanlineVisibleDots + 2 && m_showBackground && m_showSprites)
                {
                    //Set bits related to horizontal position
                    m_dataAddress &= ~0x41f; //Unset horizontal bits
                    m_dataAddress |= m_tempAddress & 0x41f; //Copy
                }
                else if (m_cycle > 280 && m_cycle <= 304 && m_showBackground && m_showSprites)
                {
                    //Set vertical bits
                    m_dataAddress &= ~0x7be0; //Unset bits related to horizontal
                    m_dataAddress |= m_tempAddress & 0x7be0; //Copy
                }
//                 if (m_cycle > 257 && m_cycle < 320)
//                     m_spriteDataAddress = 0;
               //if rendering is on, every other frame is one cycle shorter
                if (m_cycle >= ScanlineEndCycle - (!m_evenFrame && m_showBackground && m_showSprites))
                {
                    m_pipelineState = Render;
                    m_cycle = m_scanline = 0;
                }

                // add IRQ support for MMC3
                if(m_cycle==260 && m_showBackground && m_showSprites){
                    m_bus.scanlineIRQ();
                }
                break;
            case Render:
                if (m_cycle > 0 && m_cycle <= ScanlineVisibleDots)
                {
                    Byte bgColor = 0, sprColor = 0;
                    bool bgOpaque = false, sprOpaque = true;
                    bool spriteForeground = false;

                    int x = m_cycle - 1;
                    int y = m_scanline;

                    if (m_showBackground)
                    {
                        auto x_fine = (m_fineXScroll + x) % 8;
                        if (!m_hideEdgeBackground || x >= 8)
                        {
                            //fetch tile
                            auto addr = 0x2000 | (m_dataAddress & 0x0FFF); //mask off fine y
                            //auto addr = 0x2000 + x / 8 + (y / 8) * (ScanlineVisibleDots / 8);
                            Byte tile = read(addr);

                            //fetch pattern
                            //Each pattern occupies 16 bytes, so multiply by 16
                            addr = (tile * 16) + ((m_dataAddress >> 12/*y % 8*/) & 0x7); //Add fine y
                            addr |= m_bgPage << 12; //set whether the pattern is in the high or low page
                            //Get the corresponding bit determined by (8 - x_fine) from the right
                            bgColor = (read(addr) >> (7 ^ x_fine)) & 1; //bit 0 of palette entry
                            bgColor |= ((read(addr + 8) >> (7 ^ x_fine)) & 1) << 1; //bit 1

                            bgOpaque = bgColor; //flag used to calculate final pixel with the sprite pixel

                            //fetch attribute and calculate higher two bits of palette
                            addr = 0x23C0 | (m_dataAddress & 0x0C00) | ((m_dataAddress >> 4) & 0x38)
                                        | ((m_dataAddress >> 2) & 0x07);
                            auto attribute = read(addr);
                            int shift = ((m_dataAddress >> 4) & 4) | (m_dataAddress & 2);
                            //Extract and set the upper two bits for the color
                            bgColor |= ((attribute >> shift) & 0x3) << 2;
                        }
                        //Increment/wrap coarse X
                        if (x_fine == 7)
                        {
                            if ((m_dataAddress & 0x001F) == 31) // if coarse X == 31
                            {
                                m_dataAddress &= ~0x001F;          // coarse X = 0
                                m_dataAddress ^= 0x0400;           // switch horizontal nametable
                            }
                            else
                            {
                                m_dataAddress += 1;                // increment coarse X
                            }
                        }
                    }

                    if (m_showSprites && (!m_hideEdgeSprites || x >= 8))
                    {
                        for (auto i : m_scanlineSprites)
                        {
                            Byte spr_x =     m_spriteMemory[i * 4 + 3];

                            if (0 > x - spr_x || x - spr_x >= 8)
                                continue;

                            Byte spr_y     = m_spriteMemory[i * 4 + 0] + 1,
                                 tile      = m_spriteMemory[i * 4 + 1],
                                 attribute = m_spriteMemory[i * 4 + 2];

                            int length = (m_longSprites) ? 16 : 8;

                            int x_shift = (x - spr_x) % 8, y_offset = (y - spr_y) % length;

                            if ((attribute & 0x40) == 0) //If NOT flipping horizontally
                                x_shift ^= 7;
                            if ((attribute & 0x80) != 0) //IF flipping vertically
                                y_offset ^= (length - 1);

                            Address addr = 0;

                            if (!m_longSprites)
                            {
                                addr = tile * 16 + y_offset;
                                if (m_sprPage == High) addr += 0x1000;
                            }
                            else //8x16 sprites
                            {
                                //bit-3 is one if it is the bottom tile of the sprite, multiply by two to get the next pattern
                                y_offset = (y_offset & 7) | ((y_offset & 8) << 1);
                                addr = (tile >> 1) * 32 + y_offset;
                                addr |= (tile & 1) << 12; //Bank 0x1000 if bit-0 is high
                            }

                            sprColor |= (read(addr) >> (x_shift)) & 1; //bit 0 of palette entry
                            sprColor |= ((read(addr + 8) >> (x_shift)) & 1) << 1; //bit 1

                            if (!(sprOpaque = sprColor))
                            {
                                sprColor = 0;
                                continue;
                            }

                            sprColor |= 0x10; //Select sprite palette
                            sprColor |= (attribute & 0x3) << 2; //bits 2-3

                            spriteForeground = !(attribute & 0x20);

                            //Sprite-0 hit detection
                            if (!m_sprZeroHit && m_showBackground && i == 0 && sprOpaque && bgOpaque)
                            {
                                m_sprZeroHit = true;
                            }

                            break; //Exit the loop now since we've found the highest priority sprite
                        }
                    }

                    Byte paletteAddr = bgColor;

                    if ( (!bgOpaque && sprOpaque) ||
                         (bgOpaque && sprOpaque && spriteForeground) )
                        paletteAddr = sprColor;
                    else if (!bgOpaque && !sprOpaque)
                        paletteAddr = 0;
                    //else bgColor

                    m_pictureBuffer[x][y] = sf::Color(colors[m_bus.readPalette(paletteAddr)]);
                }
                else if (m_cycle == ScanlineVisibleDots + 1 && m_showBackground)
                {
                    //Shamelessly copied from nesdev wiki
                    if ((m_dataAddress & 0x7000) != 0x7000)  // if fine Y < 7
                        m_dataAddress += 0x1000;              // increment fine Y
                    else
                    {
                        m_dataAddress &= ~0x7000;             // fine Y = 0
                        int y = (m_dataAddress & 0x03E0) >> 5;    // let y = coarse Y
                        if (y == 29)
                        {
                            y = 0;                                // coarse Y = 0
                            m_dataAddress ^= 0x0800;              // switch vertical nametable
                        }
                        else if (y == 31)
                            y = 0;                                // coarse Y = 0, nametable not switched
                        else
                            y += 1;                               // increment coarse Y
                        m_dataAddress = (m_dataAddress & ~0x03E0) | (y << 5);
                                                                // put coarse Y back into m_dataAddress
                    }
                }
                else if (m_cycle == ScanlineVisibleDots + 2 && m_showBackground && m_showSprites)
                {
                    //Copy bits related to horizontal position
                    m_dataAddress &= ~0x41f;
                    m_dataAddress |= m_tempAddress & 0x41f;
                }

//                 if (m_cycle > 257 && m_cycle < 320)
//                     m_spriteDataAddress = 0;

                // add IRQ support for MMC3
                if(m_cycle==260 && m_showBackground && m_showSprites){
                    m_bus.scanlineIRQ();
                }

                if (m_cycle >= ScanlineEndCycle)
                {
                    //Find and index sprites that are on the next Scanline
                    //This isn't where/when this indexing, actually copying in 2C02 is done
                    //but (I think) it shouldn't hurt any games if this is done here

                    m_scanlineSprites.resize(0);

                    int range = 8;
                    if (m_longSprites)
                    {
                        range = 16;
                    }

                    std::size_t j = 0;
                    for (std::size_t i = m_spriteDataAddress / 4; i < 64; ++i)
                    {
                        auto diff = (m_scanline - m_spriteMemory[i * 4]);
                        if (0 <= diff && diff < range)
                        {
                            if (j >= 8)
                            {
                                m_spriteOverflow = true;
                                break;
                            }
                            m_scanlineSprites.push_back(i);
                            ++j;
                        }
                    }

                    ++m_scanline;
                    m_cycle = 0;
                }

                if (m_scanline >= VisibleScanlines)
                    m_pipelineState = PostRender;

                break;
            case PostRender:
                if (m_cycle >= ScanlineEndCycle)
                {
                    ++m_scanline;
                    m_cycle = 0;
                    m_pipelineState = VerticalBlank;

                    for (std::size_t x = 0; x < m_pictureBuffer.size(); ++x)
                    {
                        for (std::size_t y = 0; y < m_pictureBuffer[0].size(); ++y)
                        {
                            m_screen.setPixel(x, y, m_pictureBuffer[x][y]);
                        }
                    }

                }

                break;
            case VerticalBlank:
                if (m_cycle == 1 && m_scanline == VisibleScanlines + 1)
                {
                    m_vblank = true;
                    if (m_generateInterrupt) m_vblankCallback();
                }

                if (m_cycle >= ScanlineEndCycle)
                {
                    ++m_scanline;
                    m_cycle = 0;
                }

                if (m_scanline >= FrameEndScanline)
                {
                    m_pipelineState = PreRender;
                    m_scanline = 0;
                    m_evenFrame = !m_evenFrame;
                }

                break;
            default:
                LOG(Error) << "Well, this shouldn't have happened." << std::endl;
        }

        ++m_cycle;
    }







*/






