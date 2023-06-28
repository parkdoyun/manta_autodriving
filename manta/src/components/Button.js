import React from 'react';
import {StyleSheet} from 'react-native';
import {Button as PaperButton} from 'react-native-paper';

export default function Button({buttonColor, style, ...props}) {
  return (
    <PaperButton
      style={[styles.button, style]}
      labelStyle={styles.text}
      mode="contained"
      buttonColor={buttonColor}
      {...props}
    />
  );
}

const styles = StyleSheet.create({
  button: {
    width: '40%',
    alignSelf: 'center',
    marginTop: 20,
    borderRadius: 20,
  },
  text: {
    fontWeight: 'bold',
    fontSize: 20,
    lineHeight: 26,
  },
});
