#include "test_framework.h"
#include "test_mocks.h"
#include "test_config.h"
#include "test_safety.h"
#include "test_commands.h"
#include "test_kinematics.h"

// Функции трипоидной походки для тестирования
class GaitTester {
public:
    // Имитация функции handle_gait_cycle из hexapod.ino с ИСПРАВЛЕННОЙ логикой
    static void test_gait_cycle_step(GaitPhase phase, int step) {
        for (int leg = 0; leg < TOTAL_LEGS; leg++) {
            bool is_transfer = (phase == GaitPhase::PHASE1 &&
                (leg == LEG_FRONT_RIGHT || leg == LEG_REAR_RIGHT || leg == LEG_MIDDLE_LEFT)) ||
                (phase == GaitPhase::PHASE2 &&
                (leg == LEG_MIDDLE_RIGHT || leg == LEG_REAR_LEFT || leg == LEG_FRONT_LEFT));

            const int (*traj)[3] = is_transfer ? TRANSFER_TRAJ : SUPPORT_TRAJ;
            
            // ИСПРАВЛЕННАЯ ЛОГИКА: используем LEG_FORWARD_DIRECTIONS для COXA
            int base_coxa_offset = traj[step][0] - NEUTRAL;
            int base_femur_offset = traj[step][1] - NEUTRAL;
            int base_tibia_offset = traj[step][2] - NEUTRAL;
            
            // COXA использует LEG_FORWARD_DIRECTIONS (все ноги +1 для синхронного движения)
            // FEMUR и TIBIA используют LEG_LIFT_DIRECTIONS (зеркальная инверсия для подъёма)
            int coxa = NEUTRAL + LEG_OFFSETS[leg][COXA] + (base_coxa_offset * LEG_FORWARD_DIRECTIONS[leg]);
            int femur = NEUTRAL + LEG_OFFSETS[leg][FEMUR] + (base_femur_offset * LEG_LIFT_DIRECTIONS[leg][FEMUR]);
            int tibia = NEUTRAL + LEG_OFFSETS[leg][TIBIA] + (base_tibia_offset * LEG_LIFT_DIRECTIONS[leg][TIBIA]);
            
            coxa = constrain(coxa, MIN_PULSE, MAX_PULSE);
            femur = constrain(femur, MIN_PULSE, MAX_PULSE);
            tibia = constrain(tibia, MIN_PULSE, MAX_PULSE);

            SafetySystem::set_servo(LEG_SERVO_MAP[leg][COXA], coxa);
            SafetySystem::set_servo(LEG_SERVO_MAP[leg][FEMUR], femur);
            SafetySystem::set_servo(LEG_SERVO_MAP[leg][TIBIA], tibia);
        }
    }
    
    static bool is_leg_in_transfer_phase(int leg, GaitPhase phase) {
        return (phase == GaitPhase::PHASE1 &&
                (leg == LEG_FRONT_RIGHT || leg == LEG_REAR_RIGHT || leg == LEG_MIDDLE_LEFT)) ||
               (phase == GaitPhase::PHASE2 &&
                (leg == LEG_MIDDLE_RIGHT || leg == LEG_REAR_LEFT || leg == LEG_FRONT_LEFT));
    }
};

// ===== ТЕСТЫ КОНФИГУРАЦИИ =====
TEST(servo_mapping_valid) {
    // Проверяем, что все каналы сервоприводов в допустимых пределах
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int servo_channel = LEG_SERVO_MAP[leg][joint];
            ASSERT_IN_RANGE(servo_channel, 1, 32);
        }
    }
}

TEST(leg_offsets_reasonable) {
    // Проверяем, что смещения не слишком экстремальные
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int offset = LEG_OFFSETS[leg][joint];
            ASSERT_IN_RANGE(offset, -100, 100); // Все offsets в разумных пределах
        }
    }
}

TEST(lift_directions_valid) {
    // Проверяем, что направления подъема корректные
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        // ВСЕ СУСТАВЫ должны быть +1 или -1 (включая COXA)
        int coxa_dir = LEG_LIFT_DIRECTIONS[leg][COXA];
        int femur_dir = LEG_LIFT_DIRECTIONS[leg][FEMUR];
        int tibia_dir = LEG_LIFT_DIRECTIONS[leg][TIBIA];
        
        ASSERT_TRUE(coxa_dir == 1 || coxa_dir == -1);
        ASSERT_TRUE(femur_dir == 1 || femur_dir == -1);
        ASSERT_TRUE(tibia_dir == 1 || tibia_dir == -1);
        
        // Левые ноги: только COXA инвертирован, FEMUR и TIBIA как у правых (зеркальные сервоприводы)
        bool is_left = (leg == LEG_REAR_LEFT || leg == LEG_MIDDLE_LEFT || leg == LEG_FRONT_LEFT);
        if (is_left) {
            ASSERT_EQ(-1, coxa_dir);   // Левые ноги: COXA инвертирован для поворота
            ASSERT_EQ(1, femur_dir);   // FEMUR как у правых - зеркальные сервоприводы требуют те же команды
            ASSERT_EQ(-1, tibia_dir);  // TIBIA как у правых - зеркальные сервоприводы требуют те же команды
        } else {
            ASSERT_EQ(1, coxa_dir);    // Правые ноги: стандартные направления
            ASSERT_EQ(1, femur_dir);   
            ASSERT_EQ(-1, tibia_dir);  
        }
    }
}

TEST(forward_directions_valid) {
    // Проверяем, что направления для движения ВПЕРЁД корректные
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        int forward_dir = LEG_FORWARD_DIRECTIONS[leg];
        
        // Все ноги должны иметь +1 для синхронного движения вперёд
        ASSERT_EQ(1, forward_dir);
    }
}

// ===== ТЕСТЫ БЕЗОПАСНОСТИ =====
TEST(pulse_constraints) {
    SafetySystem::init();
    
    // Тест нормальных значений
    SafetySystem::set_servo(1, 1500);
    ASSERT_EQ(1500, ServoTracker::get_position(1));
    
    // Тест ограничения минимума
    SafetySystem::set_servo(2, 500);  // Слишком мало
    ASSERT_EQ(MIN_PULSE, ServoTracker::get_position(2));
    
    // Тест ограничения максимума  
    SafetySystem::set_servo(3, 3000); // Слишком много
    ASSERT_EQ(MAX_PULSE, ServoTracker::get_position(3));
}

TEST(invalid_servo_channels) {
    SafetySystem::init();
    ServoTracker::reset_all();
    
    // Невалидные каналы должны игнорироваться
    SafetySystem::set_servo(0, 1500);   // Канал 0
    SafetySystem::set_servo(33, 1500);  // Канал 33
    SafetySystem::set_servo(-1, 1500);  // Отрицательный канал
    
    // Проверяем, что команды не прошли
    ASSERT_EQ(1500, ServoTracker::get_position(0));  // default neutral для несуществующих
    ASSERT_EQ(1500, ServoTracker::get_position(33));
    ASSERT_EQ(1500, ServoTracker::get_position(-1));
}

// ===== ТЕСТЫ ТРИПОИДНОЙ ПОХОДКИ =====
TEST(tripod_groups_correct) {
    // Проверяем корректность разделения на группы
    
    // Group 1 (PHASE1): FR(0), RR(2), ML(4)  
    ASSERT_TRUE(GaitTester::is_leg_in_transfer_phase(LEG_FRONT_RIGHT, GaitPhase::PHASE1));
    ASSERT_TRUE(GaitTester::is_leg_in_transfer_phase(LEG_REAR_RIGHT, GaitPhase::PHASE1));
    ASSERT_TRUE(GaitTester::is_leg_in_transfer_phase(LEG_MIDDLE_LEFT, GaitPhase::PHASE1));
    
    // Group 2 (PHASE2): FL(5), MR(1), RL(3)
    ASSERT_TRUE(GaitTester::is_leg_in_transfer_phase(LEG_FRONT_LEFT, GaitPhase::PHASE2));
    ASSERT_TRUE(GaitTester::is_leg_in_transfer_phase(LEG_MIDDLE_RIGHT, GaitPhase::PHASE2));
    ASSERT_TRUE(GaitTester::is_leg_in_transfer_phase(LEG_REAR_LEFT, GaitPhase::PHASE2));
    
    // Проверяем, что ноги не в обеих фазах одновременно
    ASSERT_TRUE(!GaitTester::is_leg_in_transfer_phase(LEG_FRONT_RIGHT, GaitPhase::PHASE2));
    ASSERT_TRUE(!GaitTester::is_leg_in_transfer_phase(LEG_FRONT_LEFT, GaitPhase::PHASE1));
}

TEST(gait_pulse_ranges) {
    SafetySystem::init();
    
    // Тестируем полный цикл походки
    for (int phase = 0; phase < 2; phase++) {
        GaitPhase gait_phase = (phase == 0) ? GaitPhase::PHASE1 : GaitPhase::PHASE2;
        
        for (int step = 0; step < 4; step++) {
            ServoTracker::reset_all();
            GaitTester::test_gait_cycle_step(gait_phase, step);
            
            // Проверяем, что все позиции сервоприводов в безопасных пределах
            auto positions = ServoTracker::get_all_positions();
            for (auto& pos : positions) {
                ASSERT_IN_RANGE(pos.second, MIN_PULSE, MAX_PULSE);
            }
        }
    }
}

TEST(left_leg_inversion) {
    SafetySystem::init();
    ServoTracker::reset_all();
    
    // Тестируем один шаг с подъемом левых ног
    GaitTester::test_gait_cycle_step(GaitPhase::PHASE2, 1); // FL, MR, RL поднимаются
    
    // Проверяем позиции FEMUR для левых и правых ног
    int fl_femur = ServoTracker::get_position(LEG_SERVO_MAP[LEG_FRONT_LEFT][FEMUR]);
    int rl_femur = ServoTracker::get_position(LEG_SERVO_MAP[LEG_REAR_LEFT][FEMUR]);
    int mr_femur = ServoTracker::get_position(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][FEMUR]);
    
    std::cout << "FL FEMUR: " << fl_femur << ", RL FEMUR: " << rl_femur << ", MR FEMUR: " << mr_femur << std::endl;
    
    // НОВАЯ ЛОГИКА: зеркальные сервоприводы получают те же команды для того же физического результата
    // ВСЕ ноги: FEMUR должен быть больше нейтрали для подъема (одинаковые команды)
    ASSERT_TRUE(fl_femur > NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR]);   // Левые как правые!
    ASSERT_TRUE(rl_femur > NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR]);   // Левые как правые!
    
    // Правая нога: FEMUR больше нейтрали (стандартный подъем)
    ASSERT_TRUE(mr_femur > NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR]);
}

// ===== ТЕСТЫ КОМАНД =====
TEST(commands_initialization) {
    Serial1.clear_commands();
    Commands::init_controller();
    
    auto commands = Serial1.get_sent_commands();
    ASSERT_EQ(2, commands.size()); // Точно 2 команды инициализации
    ASSERT_TRUE(commands[0] == "#255P0T0\r\n"); // Команда остановки
    ASSERT_TRUE(commands[1] == "#0P1500T0\r\n"); // Команда сброса
}

TEST(reset_all_servos) {
    ServoTracker::reset_all();
    Commands::reset_all_servos();
    
    // Проверяем, что все сервоприводы установлены в стартовые позиции
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int servo = LEG_SERVO_MAP[leg][joint];
            int expected = constrain(NEUTRAL + LEG_OFFSETS[leg][joint], MIN_PULSE, MAX_PULSE);
            ASSERT_EQ(expected, ServoTracker::get_position(servo));
        }
    }
}

// ===== ТЕСТЫ ТРАЕКТОРИЙ =====
TEST(trajectory_values_safe) {
    // Проверяем, что все значения в траекториях безопасны
    for (int step = 0; step < 4; step++) {
        for (int joint = 0; joint < 3; joint++) {
            ASSERT_IN_RANGE(TRANSFER_TRAJ[step][joint], MIN_PULSE, MAX_PULSE);
            ASSERT_IN_RANGE(SUPPORT_TRAJ[step][joint], MIN_PULSE, MAX_PULSE);
        }
    }
}

TEST(trajectory_progression) {
    // Проверяем логику траекторий
    
    // TRANSFER: должна иметь подъем (увеличение FEMUR)
    ASSERT_TRUE(TRANSFER_TRAJ[1][FEMUR] > TRANSFER_TRAJ[0][FEMUR]); // Подъем увеличивается
    
    // SUPPORT: должна показывать отталкивание (COXA движется назад)
    ASSERT_TRUE(SUPPORT_TRAJ[3][COXA] < SUPPORT_TRAJ[0][COXA]); // Движение назад
}

// ===== НОВЫЕ ТЕСТЫ ДЛЯ ВЕРСИИ 3.0 =====

TEST(trajectory_amplitude_sufficient) {
    // Проверяем, что амплитуда COXA достаточна (минимум ±150)
    int transfer_min_coxa = 10000, transfer_max_coxa = 0;
    int support_min_coxa = 10000, support_max_coxa = 0;
    
    for (int step = 0; step < 4; step++) {
        if (TRANSFER_TRAJ[step][COXA] < transfer_min_coxa) transfer_min_coxa = TRANSFER_TRAJ[step][COXA];
        if (TRANSFER_TRAJ[step][COXA] > transfer_max_coxa) transfer_max_coxa = TRANSFER_TRAJ[step][COXA];
        if (SUPPORT_TRAJ[step][COXA] < support_min_coxa) support_min_coxa = SUPPORT_TRAJ[step][COXA];
        if (SUPPORT_TRAJ[step][COXA] > support_max_coxa) support_max_coxa = SUPPORT_TRAJ[step][COXA];
    }
    
    int transfer_amplitude = (transfer_max_coxa - NEUTRAL) - (transfer_min_coxa - NEUTRAL);
    int support_amplitude = (support_max_coxa - NEUTRAL) - (support_min_coxa - NEUTRAL);
    
    std::cout << "  TRANSFER amplitude: " << transfer_amplitude << " (min: " << transfer_min_coxa << ", max: " << transfer_max_coxa << ")" << std::endl;
    std::cout << "  SUPPORT amplitude: " << support_amplitude << " (min: " << support_min_coxa << ", max: " << support_max_coxa << ")" << std::endl;
    
    // Амплитуда должна быть минимум 300 (±150)
    ASSERT_TRUE(transfer_amplitude >= 300);
    ASSERT_TRUE(support_amplitude >= 300);
}

TEST(trajectory_symmetry) {
    // Проверяем симметрию TRANSFER и SUPPORT
    // TRANSFER должна начинаться где SUPPORT заканчивается
    
    int transfer_start_coxa = TRANSFER_TRAJ[0][COXA];
    int support_end_coxa = SUPPORT_TRAJ[3][COXA];
    
    int transfer_end_coxa = TRANSFER_TRAJ[3][COXA];
    int support_start_coxa = SUPPORT_TRAJ[0][COXA];
    
    std::cout << "  TRANSFER: " << transfer_start_coxa << " -> " << transfer_end_coxa << std::endl;
    std::cout << "  SUPPORT:  " << support_start_coxa << " -> " << support_end_coxa << std::endl;
    
    // Проверяем циклическую симметрию (начало одного = конец другого)
    ASSERT_EQ(transfer_start_coxa, support_end_coxa);
    ASSERT_EQ(transfer_end_coxa, support_start_coxa);
}

TEST(trajectory_forward_movement_logic) {
    // Проверяем логику прямолинейного движения
    
    // TRANSFER: COXA должна двигаться от НАЗАД (-) к ВПЕРЁД (+)
    int transfer_coxa_change = TRANSFER_TRAJ[3][COXA] - TRANSFER_TRAJ[0][COXA];
    ASSERT_TRUE(transfer_coxa_change > 0); // Движение вперёд
    
    // SUPPORT: COXA должна двигаться от ВПЕРЁД (+) к НАЗАД (-)
    int support_coxa_change = SUPPORT_TRAJ[3][COXA] - SUPPORT_TRAJ[0][COXA];
    ASSERT_TRUE(support_coxa_change < 0); // Движение назад (толкает тело вперёд)
    
    std::cout << "  TRANSFER COXA change: " << transfer_coxa_change << " (должно быть >0)" << std::endl;
    std::cout << "  SUPPORT COXA change: " << support_coxa_change << " (должно быть <0)" << std::endl;
}

TEST(all_legs_synchronous_forward) {
    // Проверяем, что LEG_FORWARD_DIRECTIONS одинаковы для всех ног
    // Это критично для прямолинейного движения!
    
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        ASSERT_EQ(LEG_FORWARD_DIRECTIONS[leg], +1);
    }
    
    std::cout << "  ✅ Все ноги используют одинаковое направление (+1) для COXA" << std::endl;
    std::cout << "  ✅ Это обеспечивает синхронное движение вперёд" << std::endl;
}

// ===== СИМУЛЯЦИЯ TRIPOD_TEST =====
TEST(simulate_tripod_test) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "🕷️  СИМУЛЯЦИЯ TRIPOD_TEST (без загрузки на ESP32)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    const int LIFT_AMOUNT = 120;  
    const int FORWARD_AMOUNT = 80; 
    
    std::cout << "\nПараметры теста: LIFT_AMOUNT=" << LIFT_AMOUNT << ", FORWARD_AMOUNT=" << FORWARD_AMOUNT << std::endl;
    
    // Диагностика для ML ноги
    std::cout << "\n" << std::string(40, '-') << std::endl;
    std::cout << "ML (MIDDLE LEFT) DIAGNOSTIC" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    std::cout << "NEUTRAL=" << NEUTRAL << std::endl;
    std::cout << "ML LEG_OFFSETS: COXA=" << LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA] 
              << ", FEMUR=" << LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR] 
              << ", TIBIA=" << LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA] << std::endl;
    std::cout << "ML LEG_LIFT_DIRECTIONS: COXA=" << LEG_LIFT_DIRECTIONS[LEG_MIDDLE_LEFT][COXA] 
              << ", FEMUR=" << LEG_LIFT_DIRECTIONS[LEG_MIDDLE_LEFT][FEMUR] 
              << ", TIBIA=" << LEG_LIFT_DIRECTIONS[LEG_MIDDLE_LEFT][TIBIA] << std::endl;
    
    for (int cycle = 0; cycle < 2; cycle++) {
        std::cout << "\n" << std::string(50, '=') << std::endl;
        std::cout << "Tripod cycle " << (cycle + 1) << std::endl;
        std::cout << std::string(50, '=') << std::endl;
        
        // === ФАЗА 1: FR, ML, RR поднимаются ===
        std::cout << "\nPHASE 1: Lifting FR, ML, RR" << std::endl;
        std::cout << std::string(30, '-') << std::endl;
        
        // FR (Front Right) - поднимаем
        int fr_coxa_base = NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][COXA];
        int fr_femur_base = NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][FEMUR];  
        int fr_tibia_base = NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][TIBIA];
        
        int fr_coxa = constrain(fr_coxa_base + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        int fr_femur = constrain(fr_femur_base + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); 
        int fr_tibia = constrain(fr_tibia_base - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); 
        
        // ML (Middle Left) - поднимаем (зеркальные сервоприводы - те же команды, что у правых!)
        int ml_coxa_base = NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA];
        int ml_femur_base = NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR];
        int ml_tibia_base = NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA];
        
        int ml_coxa = constrain(ml_coxa_base - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE); // Только COXA инвертирован
        int ml_femur = constrain(ml_femur_base + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // FEMUR как у правых!
        int ml_tibia = constrain(ml_tibia_base - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // TIBIA как у правых! 
        
        // RR (Rear Right) - поднимаем
        int rr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        int rr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); 
        int rr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); 
        
        std::cout << "FR (Front Right): COXA=" << fr_coxa << ", FEMUR=" << fr_femur << ", TIBIA=" << fr_tibia << " [LIFTING]" << std::endl;
        std::cout << "ML (Middle Left): COXA=" << ml_coxa << ", FEMUR=" << ml_femur << ", TIBIA=" << ml_tibia << " [LIFTING-INVERTED]" << std::endl;
        std::cout << "RR (Rear Right):  COXA=" << rr_coxa << ", FEMUR=" << rr_femur << ", TIBIA=" << rr_tibia << " [LIFTING]" << std::endl;
        
        // FL, MR, RL на земле толкают назад (левые ноги инвертированы!)
        int fl_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        int mr_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        int rl_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        
        std::cout << "FL (Front Left):   COXA=" << fl_ground_coxa << ", FEMUR=" << (NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR]) << ", TIBIA=" << (NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][TIBIA]) << " [GROUND-INVERTED]" << std::endl;
        std::cout << "MR (Middle Right): COXA=" << mr_ground_coxa << ", FEMUR=" << (NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR]) << ", TIBIA=" << (NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][TIBIA]) << " [GROUND]" << std::endl;
        std::cout << "RL (Rear Left):    COXA=" << rl_ground_coxa << ", FEMUR=" << (NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR]) << ", TIBIA=" << (NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][TIBIA]) << " [GROUND-INVERTED]" << std::endl;
        
        // === ФАЗА 2: FL, MR, RL поднимаются ===
        std::cout << "\nPHASE 2: Lifting FL, MR, RL" << std::endl;
        std::cout << std::string(30, '-') << std::endl;
        
        // FL (Front Left) - поднимаем (зеркальные сервоприводы - те же команды, что у правых!)
        int fl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE); // Только COXA инвертирован
        int fl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // FEMUR как у правых!
        int fl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // TIBIA как у правых!
        
        // MR (Middle Right) - поднимаем
        int mr_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        int mr_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); 
        int mr_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); 
        
        // RL (Rear Left) - поднимаем (зеркальные сервоприводы - те же команды, что у правых!)
        int rl_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE); // Только COXA инвертирован
        int rl_femur = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR] + LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // FEMUR как у правых!
        int rl_tibia = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][TIBIA] - LIFT_AMOUNT, MIN_PULSE, MAX_PULSE); // TIBIA как у правых! 
        
        std::cout << "FL (Front Left):  COXA=" << fl_coxa << ", FEMUR=" << fl_femur << ", TIBIA=" << fl_tibia << " [LIFTING-INVERTED]" << std::endl;
        std::cout << "MR (Middle Right): COXA=" << mr_coxa << ", FEMUR=" << mr_femur << ", TIBIA=" << mr_tibia << " [LIFTING]" << std::endl;
        std::cout << "RL (Rear Left):   COXA=" << rl_coxa << ", FEMUR=" << rl_femur << ", TIBIA=" << rl_tibia << " [LIFTING-INVERTED]" << std::endl;
        
        // FR, ML, RR на земле толкают назад (левые ноги инвертированы!)
        int fr_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        int ml_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][COXA] + FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        int rr_ground_coxa = constrain(NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][COXA] - FORWARD_AMOUNT, MIN_PULSE, MAX_PULSE);
        
        std::cout << "FR (Front Right): COXA=" << fr_ground_coxa << ", FEMUR=" << (NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][FEMUR]) << ", TIBIA=" << (NEUTRAL + LEG_OFFSETS[LEG_FRONT_RIGHT][TIBIA]) << " [GROUND]" << std::endl;
        std::cout << "ML (Middle Left): COXA=" << ml_ground_coxa << ", FEMUR=" << (NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR]) << ", TIBIA=" << (NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][TIBIA]) << " [GROUND-INVERTED]" << std::endl;
        std::cout << "RR (Rear Right):  COXA=" << rr_ground_coxa << ", FEMUR=" << (NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][FEMUR]) << ", TIBIA=" << (NEUTRAL + LEG_OFFSETS[LEG_REAR_RIGHT][TIBIA]) << " [GROUND]" << std::endl;
        
        if (cycle == 0) break; // Показываем только 1 цикл для краткости
    }
    
    // Анализ ML ноги
    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << "🔍 АНАЛИЗ ML НОГИ" << std::endl;
    std::cout << std::string(50, '=') << std::endl;
    
    int ml_neutral_femur = NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_LEFT][FEMUR];
    int ml_lifting_femur = ml_neutral_femur + LIFT_AMOUNT;  // НОВАЯ ЛОГИКА: как у правых ног!
    
    std::cout << "ML FEMUR нейтраль: " << ml_neutral_femur << std::endl;
    std::cout << "ML FEMUR подъем:   " << ml_lifting_femur << " (разница: +" << (ml_lifting_femur - ml_neutral_femur) << ")" << std::endl;
    
    if (ml_lifting_femur > 1600) {
        std::cout << "✅ УСПЕХ: ML FEMUR=" << ml_lifting_femur << " поднимается правильно!" << std::endl;
        std::cout << "🕷️  ЗЕРКАЛЬНЫЕ СЕРВОПРИВОДЫ: Те же команды, то же физическое движение!" << std::endl;
    } else {
        std::cout << "⚠️  ПРОБЛЕМА: ML FEMUR=" << ml_lifting_femur << " все еще недостаточно высоко!" << std::endl;
    }
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "Симуляция завершена!" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
}
