#include <fstream>
#include <type_traits>

namespace soundlink {

template<typename T>
struct file_reader : runnable {
    static_assert(std::is_trivially_copyable_v<T>,"file_reader requires a trivially copyable type");

    file_reader(scheduler* sch, const char* filename, pipebuf<T>& _out)
        :runnable(sch, _out.name), file(filename, std::ios::binary), out(_out) {
        if (!file)
            fail("could not open input file");
    }

    void run() override {
        const size_t count = out.writable();
        if (count == 0 || file.eof())
            return;

        file.read(
            reinterpret_cast<char*>(out.wr()),
            static_cast<std::streamsize>(count * sizeof(T))
        );

        const std::streamsize bytes = file.gcount();

        if (file.bad())
            fail("error reading input file");

        if (bytes % sizeof(T) != 0)
            fail("input file ends with a partial element");

        out.written(
            static_cast<unsigned long>(bytes / sizeof(T))
        );
    }

private:
    std::ifstream file;
    pipewriter<T> out;
};


template<typename T>
struct file_writer : runnable {
    static_assert(std::is_trivially_copyable_v<T>, "file_writer requires a trivially copyable type");

    file_writer(scheduler* sch,
                pipebuf<T>& _in,
                const char* filename)
        : runnable(sch, _in.name),
          in(_in),
          file(filename, std::ios::binary) {
        if (!file)
            fail("could not open output file");
    }

    void run() override {
        const unsigned long count = in.readable();
        if (count == 0)
            return;

        file.write(
            reinterpret_cast<const char*>(in.rd()),
            static_cast<std::streamsize>(count * sizeof(T))
        );

        if (!file)
            fail("error writing output file");

        in.read(count);
    }

    void shutdown() override {
        file.flush();

        if (!file)
            fail("error flushing output file");
    }

private:
    pipereader<T> in;
    std::ofstream file;
};

} // namespace soundlink