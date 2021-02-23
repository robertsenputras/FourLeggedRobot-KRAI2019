#include "encoderHAL.h"

namespace mbed
{   

    encoderHAL::encoderHAL(TIM_TypeDef * _TIM)
    {
        TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        EncoderInit(&encoder, &timer, _TIM, 0xffff, TIM_ENCODERMODE_TI12);

        // Update (aka over- and underflow) interrupt enabled
        TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        TIM->SR &= 0xfffe;
        //generate update event
        TIM->EGR = 1;
        //enable counter
        TIM->CR1 = 1;
        
    }
    
    encoderHAL::encoderHAL(TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode)
    {
        TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        EncoderInit(&encoder, &timer, _TIM, _maxcount, _encmode);

        // Update (aka over- and underflow) interrupt enabled
        TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        TIM->SR &= 0xfffe;
    }
    
    encoderHAL::encoderHAL(TIM_Encoder_InitTypeDef * _encoder, TIM_HandleTypeDef * _timer, TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode)
    {
        timer = *_timer;
        encoder = *_encoder;
        TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        EncoderInit(&encoder, &timer, _TIM, _maxcount, _encmode);

        // Update (aka over- and underflow) interrupt enabled
        TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        TIM->SR &= 0xfffe;
    }

    
    int32_t encoderHAL::getPulses(bool reset)
    {
        int16_t count = TIM->CNT;
        if(reset){
            switch((uint32_t)TIM){
                case TIM1_BASE :
                    TIM1->CNT = 0;
                break;
                
                case TIM2_BASE :
                    TIM2->CNT = 0;
                break;
                
                case TIM3_BASE :
                    TIM3->CNT = 0;
                break;
                
                case TIM4_BASE :
                    TIM4->CNT = 0;
                break;
                
                case TIM5_BASE :
                    TIM5->CNT = 0;
                break;
                
                case TIM8_BASE :
                    TIM8->CNT = 0;
                break;
            }
        }
        else{
            switch((uint32_t)TIM)
            {
                case TIM1_BASE :
                    return (int32_t)count;
                
                case TIM2_BASE :
                    return (int32_t)count;
                
                case TIM3_BASE :
                    return (int32_t)count;
                
                case TIM4_BASE :
                    return (int32_t)count;
                
                case TIM5_BASE :
                    return (int32_t)count;
                    
                case TIM8_BASE :
                    return (int32_t)count;
            }
        }
        
        return (int32_t)count;
    }
    
    
    TIM_HandleTypeDef* encoderHAL::GetTimer()
    {
        return &timer;    
    }
    
}