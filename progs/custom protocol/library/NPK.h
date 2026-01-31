#ifndef NPK_H
#define NPK_H

#include <CRC.h>
#include <SoftwareSerial.h>

class DataPack {
	// [Start Marker][Command][Data Type Flags][Payload Length][Payload][CRC] - структура пакета
	public:
    static inline const size_t MAX_STRING_SIZE = 32;
    static inline const size_t MAX_ARGS_COUNT = 10;
    static inline const size_t MAX_PAYLOAD_SIZE = MAX_ARGS_COUNT * MAX_STRING_SIZE;
    static inline const size_t MIN_PACK_SIZE = 7;
    static inline const size_t MAX_PACK_SIZE = MIN_PACK_SIZE + MAX_PAYLOAD_SIZE + MAX_ARGS_COUNT;

    enum class Command : uint8_t {
      RESPONSE = 0x00,
      BLINK    = 0x01,
      LCD      = 0x02,
      GO       = 0x03,
      GETRANGE = 0X04,
      GETSPEED = 0X05,
      // ...
    };

    enum class ArgType : uint8_t {
			BOOL    = 0x01, // 1
			UINT8   = 0x02, // 1
			INT16   = 0x03, // 2
			UINT16  = 0x04, // 2
			FLOAT   = 0x05, // 4
			STRING  = 0x06  // N+1
    };

    static uint16_t calculate_crc(const uint8_t* data, size_t data_size);

    DataPack(Command command);

		//Десериализует сырые данные
    explicit DataPack(SoftwareSerial& serial);
    

    DataPack(const DataPack&) = delete;
    DataPack& operator=(const DataPack&) = delete;
    DataPack(DataPack&& other) noexcept;
    ~DataPack();

		// Выдаёт размер массива сырых данных
    size_t get_serialized_size() const;

		// Выдает массив сырых данных
    uint8_t* get_serialized_data() const;

		// Перегрузки append_arg
    // Добавляет type в args_types_ и закодированный value в payload_
    // Увеличивает размерные поля - args_count и payload_length_
    void append_arg(bool value);
    void append_arg(uint8_t value);
    void append_arg(int16_t value);
    void append_arg(uint16_t value);
    void append_arg(float value);
    void append_arg(const char* value);

		// Выполняет команду пакета
    // Возвращает response в виде DataPack, где в поле command_ лежит Command::RESPONSE
    // реализация метода задумана в коде slave-устройства (см examples)
    DataPack perform_command();

    bool is_valid() const;
    template<typename T> T get_next_val();
    size_t get_args_count() const;
    
	private:
    Command command_;
    size_t args_count_;
    ArgType* args_types_;
    size_t payload_length_;
    uint8_t* payload_;
    mutable size_t current_payload_index_; 
    bool is_valid_;

    
};

// Шаблонные "перегрузки" get_next_val()
// Выдаёт аргумент соответствующий type по данным на которые указывает current_payload_index_
// Сдвигает current_payload_index_ на начало данных следующего аргумента
// !! (пока) Проверки на соотвествие запрошенного аргумента и реального
template<> bool DataPack::get_next_val<bool>();
template<> uint8_t DataPack::get_next_val<uint8_t>();
template<> int16_t DataPack::get_next_val<int16_t>();
template<> uint16_t DataPack::get_next_val<uint16_t>();
template<> float DataPack::get_next_val<float>();
template<> const char* DataPack::get_next_val<const char*>();

#endif // NPK_H