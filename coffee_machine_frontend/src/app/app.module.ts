import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppComponent } from './app.component';
import { NZ_I18N } from 'ng-zorro-antd/i18n';
import { en_US } from 'ng-zorro-antd/i18n';
import { registerLocaleData } from '@angular/common';
import en from '@angular/common/locales/en';
import { FormsModule } from '@angular/forms';
import { HttpClientModule } from '@angular/common/http';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { NzCardModule } from 'ng-zorro-antd/card';
import { NzButtonModule } from 'ng-zorro-antd/button'
import { CafeComponent } from './cafe/cafe.component'

registerLocaleData(en);

const firebaseConfig = {
  apiKey: "AIzaSyDmChXDoPxRTzKjUfM5ZrCFf7OTddv_tVs",
  authDomain: "sistemas-embarcados-1.firebaseapp.com",
  databaseURL: "https://sistemas-embarcados-1-default-rtdb.firebaseio.com",
  projectId: "sistemas-embarcados-1",
  storageBucket: "sistemas-embarcados-1.appspot.com",
  messagingSenderId: "552043667147",
  appId: "1:552043667147:web:62b8f257708b6bfcd4454e",
  measurementId: "G-71HSXDVBP7"
};

@NgModule({
  declarations: [
    AppComponent,
    CafeComponent
  ],
  imports: [
    BrowserModule,
    FormsModule,
    HttpClientModule,
    BrowserAnimationsModule,
    NzCardModule,
    NzButtonModule,
    HttpClientModule
  ],
  providers: [{ provide: NZ_I18N, useValue: en_US }],
  bootstrap: [AppComponent]
})
export class AppModule { }
