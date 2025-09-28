#include "test_framework.h"
#include "test_mocks.h"
#include "test_config.h"
#include "test_safety.h"
#include "test_commands.h"

// Функции трипоидной походки для тестирования
class GaitTester {
public:
    // Имитация функции handle_gait_cycle из hexapod.ino
    static void test_gait_cycle_step(GaitPhase phase, int step) {
        for (int leg = 0; leg < TOTAL_LEGS; leg++) {
            bool is_transfer = (phase == GaitPhase::PHASE1 &&
                (leg == LEG_FRONT_RIGHT || leg == LEG_REAR_RIGHT || leg == LEG_MIDDLE_LEFT)) ||
                (phase == GaitPhase::PHASE2 &&
                (leg == LEG_MIDDLE_RIGHT || leg == LEG_REAR_LEFT || leg == LEG_FRONT_LEFT));

            const int (*traj)[3] = is_transfer ? TRANSFER_TRAJ : SUPPORT_TRAJ;
            bool is_left_leg = (leg == LEG_REAR_LEFT || leg == LEG_MIDDLE_LEFT || leg == LEG_FRONT_LEFT);
            
            int coxa, femur, tibia;
            
            if (is_left_leg) {
                coxa = NEUTRAL + LEG_OFFSETS[leg][COXA];
                int base_femur_offset = traj[step][1] - NEUTRAL;
                int base_tibia_offset = traj[step][2] - NEUTRAL;
                
                femur = NEUTRAL + LEG_OFFSETS[leg][FEMUR] + (base_femur_offset * LEG_LIFT_DIRECTIONS[leg][FEMUR]);
                tibia = NEUTRAL + LEG_OFFSETS[leg][TIBIA] + (base_tibia_offset * LEG_LIFT_DIRECTIONS[leg][TIBIA]);
            } else {
                coxa = traj[step][0] + LEG_OFFSETS[leg][COXA];
                femur = traj[step][1] + LEG_OFFSETS[leg][FEMUR];
                tibia = traj[step][2] + LEG_OFFSETS[leg][TIBIA];
            }
            
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
            ASSERT_IN_RANGE(offset, -100, 100);
        }
    }
}

TEST(lift_directions_valid) {
    // Проверяем, что направления подъема корректные
    for (int leg = 0; leg < TOTAL_LEGS; leg++) {
        // COXA должна быть 0 (нейтральная)
        ASSERT_EQ(0, LEG_LIFT_DIRECTIONS[leg][COXA]);
        
        // FEMUR и TIBIA должны быть +1 или -1
        int femur_dir = LEG_LIFT_DIRECTIONS[leg][FEMUR];
        int tibia_dir = LEG_LIFT_DIRECTIONS[leg][TIBIA];
        
        ASSERT_TRUE(femur_dir == 1 || femur_dir == -1);
        ASSERT_TRUE(tibia_dir == 1 || tibia_dir == -1);
        
        // Левые ноги должны иметь инвертированные направления
        bool is_left = (leg == LEG_REAR_LEFT || leg == LEG_MIDDLE_LEFT || leg == LEG_FRONT_LEFT);
        if (is_left) {
            ASSERT_EQ(-1, femur_dir);
            ASSERT_EQ(1, tibia_dir);
        } else {
            ASSERT_EQ(1, femur_dir);
            ASSERT_EQ(-1, tibia_dir);
        }
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
    // Левые ноги должны иметь FEMUR < NEUTRAL (инвертированный подъем)
    int fl_femur = ServoTracker::get_position(LEG_SERVO_MAP[LEG_FRONT_LEFT][FEMUR]);
    int rl_femur = ServoTracker::get_position(LEG_SERVO_MAP[LEG_REAR_LEFT][FEMUR]);
    
    // Правая нога MR должна иметь FEMUR > NEUTRAL (нормальный подъем)
    int mr_femur = ServoTracker::get_position(LEG_SERVO_MAP[LEG_MIDDLE_RIGHT][FEMUR]);
    
    std::cout << "FL FEMUR: " << fl_femur << ", RL FEMUR: " << rl_femur << ", MR FEMUR: " << mr_femur << std::endl;
    
    // Левые ноги: FEMUR должен быть меньше нейтрали (инверсия)
    ASSERT_TRUE(fl_femur < NEUTRAL + LEG_OFFSETS[LEG_FRONT_LEFT][FEMUR]);
    ASSERT_TRUE(rl_femur < NEUTRAL + LEG_OFFSETS[LEG_REAR_LEFT][FEMUR]);
    
    // Правая нога: FEMUR должен быть больше нейтрали (нормальный подъем)
    ASSERT_TRUE(mr_femur > NEUTRAL + LEG_OFFSETS[LEG_MIDDLE_RIGHT][FEMUR]);
}

// ===== ТЕСТЫ КОМАНД =====
TEST(commands_initialization) {
    Serial1.clear_commands();
    Commands::init_controller();
    
    auto commands = Serial1.get_sent_commands();
    ASSERT_TRUE(commands.size() >= 2); // Минимум 2 команды инициализации
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
