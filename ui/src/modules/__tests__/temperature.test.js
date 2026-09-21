import { describe, it, expect } from 'vitest'
import { toDisplayTemp, fromDisplayTemp, displayTemp, deviceTemp } from '../temperature'

describe('temperature helpers', () => {
  describe('toDisplayTemp / fromDisplayTemp', () => {
    it('leaves Celsius unchanged', () => {
      expect(toDisplayTemp(19.5, 'C')).toBe(19.5)
      expect(fromDisplayTemp(19.5, 'C')).toBe(19.5)
    })

    it('treats an unset format as Celsius', () => {
      expect(toDisplayTemp(19.5, '')).toBe(19.5)
      expect(toDisplayTemp(19.5, undefined)).toBe(19.5)
      expect(fromDisplayTemp(19.5, '')).toBe(19.5)
    })

    it('converts Celsius to Fahrenheit', () => {
      expect(toDisplayTemp(0, 'F')).toBe(32)
      expect(toDisplayTemp(20, 'F')).toBe(68)
      expect(toDisplayTemp(100, 'F')).toBe(212)
      expect(toDisplayTemp(-40, 'F')).toBe(-40)
    })

    it('converts Fahrenheit to Celsius', () => {
      expect(fromDisplayTemp(32, 'F')).toBe(0)
      expect(fromDisplayTemp(68, 'F')).toBe(20)
      expect(fromDisplayTemp(212, 'F')).toBe(100)
      expect(fromDisplayTemp(-40, 'F')).toBe(-40)
    })

    it('round trips', () => {
      for (const c of [0, 4.5, 10.4, 18, 20.1, 30]) {
        expect(fromDisplayTemp(toDisplayTemp(c, 'F'), 'F')).toBeCloseTo(c, 10)
      }
    })
  })

  describe('displayTemp', () => {
    it('returns the Celsius value untouched, whatever its precision', () => {
      expect(displayTemp(9.53, 'C')).toBe(9.53)
      expect(displayTemp(20, 'C')).toBe(20)
    })

    it('returns Fahrenheit with one decimal', () => {
      expect(displayTemp(20, 'F')).toBe(68)
      expect(displayTemp(9.5, 'F')).toBe(49.1)
      expect(displayTemp(10.4, 'F')).toBe(50.7)
    })

    it('gives the range limits of the target temperature', () => {
      expect(displayTemp(0, 'C')).toBe(0)
      expect(displayTemp(30, 'C')).toBe(30)
      expect(displayTemp(0, 'F')).toBe(32)
      expect(displayTemp(30, 'F')).toBe(86)
    })
  })

  describe('deviceTemp', () => {
    it('sends Celsius values untouched, also strings', () => {
      expect(deviceTemp(20, 'C')).toBe(20)
      expect(deviceTemp('20.5', 'C')).toBe('20.5')
    })

    it('converts Fahrenheit input to Celsius with two decimals', () => {
      expect(deviceTemp(68, 'F')).toBe(20)
      expect(deviceTemp(50.7, 'F')).toBe(10.39)
      expect(deviceTemp('68', 'F')).toBe(20)
    })
  })
})
