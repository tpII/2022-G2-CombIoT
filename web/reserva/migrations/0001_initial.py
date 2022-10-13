# Generated by Django 3.1.2 on 2020-12-04 01:02

from django.db import migrations, models


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Reserva',
            fields=[
                ('id', models.AutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('codigo', models.ImageField(upload_to='')),
                ('nombre', models.CharField(max_length=20)),
                ('apellido', models.CharField(max_length=20)),
                ('DNI', models.IntegerField()),
                ('fecha', models.DateField()),
                ('email', models.EmailField(max_length=254)),
                ('hora', models.TimeField()),
            ],
        ),
    ]